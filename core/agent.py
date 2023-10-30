import os, sys
import pickle
import numpy as np
import asyncio
import copy
import logging
import re
import time

from core.action_interface import ActionInterface

from sparc_planning.src.beam_domain_coarse import generate_coarse_beam_domain
from sparc_planning.src.beam_domain_fine import generate_fine_beam_domain
from sparc_planning.src.beam_domain_abstract import generate_abstract_beam_domain
from sparc_planning.src.al_structures import ActionInstance, GoalDefinition
from sparc_planning.src.planning import plan
from sparc_planning.src.sparc_io import extract_states_from_answer_set
from sparc_planning.src.zooming import remove_chars_from_last_number, zoom
from beam_assembly.beam_assembly_parser import load_beam_xml, load_assembly_xml
from beam_assembly.beam_to_sparc import create_sparc_data
from beam_assembly.beam_insertion import (
    get_target_connection,
    calculate_insertion_poses,
    calculate_approach,
    get_insert_end,
)


MIN_COARSE_PLAN_LENGTH = 50
MAX_COARSE_PLAN_LENGTH = 55
FINE_ITERATION_MIN = 1
FINE_ITERATION_LIMIT = 10
COARSE_ITERATION_MIN = 5
COARSE_PLAN_ITERATION_LIMIT = 32

# set up logging
logger = logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(filename)s:%(lineno)s - %(levelname)s - %(message)s",
    datefmt="%m-%d %H:%M",
)


class Agent:
    def __init__(self, beam_xml, assembly_xml, domain_generator, fixed_beam_id="b7"):
        """
        Args:
            beam_xml: A xml file containing a set of beams
            assembly_xml: A xml file for a task specification
            domain_generator: A function that generates an example domain of the task
        """

        # interface for primitive actions
        self.fixed_beam_id = fixed_beam_id
        self.interface = ActionInterface(fixed_beam_id)

        # grep template
        self.action_template = "occurs\((.*),(.*)\)"
        self.abst_action_template = "(.*)\((.*)\)"

        # generate beam domain for abstract, coarse, fine-level plannig
        self.abstract = generate_abstract_beam_domain()
        self.coarse = generate_coarse_beam_domain()
        self.fine = generate_fine_beam_domain()

        # load beam information
        self.beams = load_beam_xml(beam_xml)

        # load assembly information
        self.assembly = load_assembly_xml(self.beams, assembly_xml)

        (
            coarse_sorts,
            self.xml_coarse_statics,
            fine_sorts,
            self.xml_fine_statics,
        ) = create_sparc_data(self.assembly)

        coarse_sort_dict = dict(zip([s.name for s in coarse_sorts], coarse_sorts))
        fine_sort_dict = dict(zip([s.name for s in fine_sorts], fine_sorts))

        logging.debug(f"Fine sorts: {[s.name for s in fine_sorts]}")

        # override abstract sorts with xml derived data
        for i, sort in enumerate(self.abstract.sorts):
            if sort.name in coarse_sort_dict.keys():
                self.abstract.sorts[i].instances = coarse_sort_dict[sort.name].instances
        # override coarse sorts with xml derived data
        for i, sort in enumerate(self.coarse.sorts):
            if sort.name in coarse_sort_dict.keys():
                self.coarse.sorts[i].instances = coarse_sort_dict[sort.name].instances
        # override fine sorts with xml derived data
        for i, sort in enumerate(self.fine.sorts):
            if sort.name in fine_sort_dict.keys():
                self.fine.sorts[i].instances = fine_sort_dict[sort.name].instances

        self.abstract.domain_setup = [
            f"holds(in_assembly_c({self.assembly.base.name}),true,0)."
        ] + self.xml_coarse_statics

        (
            self.coarse_fluents,
            example_coarse_statics,
            self.fine_fluents,
            example_fine_statics,
        ) = domain_generator()

        self.coarse_statics = self.xml_coarse_statics + example_coarse_statics
        self.fine_statics = self.xml_fine_statics + example_fine_statics

        self.coarse.domain_setup = self.coarse_fluents + self.coarse_statics
        self.fine.domain_setup = self.fine_fluents + self.fine_statics

        # automatically generate goal definition for abstract level (add all beams to assembly)
        for beam in self.assembly.beams:
            if beam == self.assembly.base:
                continue
            self.abstract.goal_description.append(
                GoalDefinition("in_assembly_c", [beam.name], True)
            )

        self.abstract_prog = self.abstract.to_sparc_program()

        self.abs_prog = self.abstract.to_sparc_program()
        self.abs_prog.save(
            os.path.join(
                os.environ["PLANNER_PATH"], "sparc_planning/sparc_files/abs_temp.sp"
            )
        )

    async def run_abstract_plan(self):
        """
        Run abstract-level plannig
        """
        abstract_plan = await plan(
            os.path.join(
                os.environ["PLANNER_PATH"], "sparc_planning/sparc_files/abs_temp.sp"
            ),
            max_length=len(self.assembly.beams),
            min_length=len(self.assembly.beams) - 1,
        )
        # collect results
        abstract_states, abstract_actions = extract_states_from_answer_set(
            abstract_plan[0]
        )  # [0] just takes first answer set (valid plan), further work could explore which route to take
        return abstract_states, abstract_actions

    async def run_coarse_plan(self, abstract_states, abstract_actions):
        """
        run coarse-level plannig
        args:
            abstract_states: a list of abstract-level states
            abstract_actions: a list of abstract-level actions
        """
        all_coarse_actions = []
        all_coarse_states = []
        coarse_goals = []
        coarse_plan_length = 0

        coarse_state_fluents = copy.deepcopy(self.coarse_fluents)

        for step in abstract_actions:
            inserted_beam = re.split("\(|\)|\,", step)[2]
            coarse_goals.append(
                [GoalDefinition("in_assembly_c", [inserted_beam], True)]
            )
        fastening_goal = []

        for i in range(len(self.assembly.connections)):
            conn = self.assembly.connections[i]
            B1 = conn.element1.name
            B2 = conn.element2.name
            fastening_goal.append(GoalDefinition("fastened_c", [B1, B2, f"P{i}"], True))
        coarse_goals.append(fastening_goal)

        for goal in coarse_goals:
            # extend goal
            self.coarse.goal_description += goal
            coarse_prog = self.coarse.to_sparc_program()
            coarse_prog.save(
                os.path.join(
                    os.environ["PLANNER_PATH"], "sparc_planning/sparc_files/temp.sp"
                )
            )
            # run coarse planner
            coarse_plan = await plan(
                os.path.join(
                    os.environ["PLANNER_PATH"], "sparc_planning/sparc_files/temp.sp"
                ),
                max_length=coarse_plan_length + COARSE_PLAN_ITERATION_LIMIT,
                min_length=coarse_plan_length + COARSE_ITERATION_MIN,
            )
            # collect results
            coarse_states, coarse_actions = extract_states_from_answer_set(
                coarse_plan[0], min_time_step=coarse_plan_length
            )  # [0] just takes first answer set (valid plan), further work could explore which route to take
            logging.info(coarse_actions)
            coarse_plan_length += len(coarse_actions)
            all_coarse_actions += coarse_actions
            all_coarse_states += coarse_states[
                :-1
            ]  # last state will be same as first state of next loop
            # override state for next iteration start
            self.coarse.domain_setup = coarse_states[-1].fluents + self.coarse_statics
            self.coarse.start_step = coarse_plan_length
        all_coarse_states.append(coarse_states[-1])  # add end state data
        return all_coarse_states, all_coarse_actions

    async def run_fine_plan(self, coarse_states, coarse_actions):
        """
        run fine-level plannig
        args:
            coarse_states: a list of coarse-level states
            coarse_actions: a list of coarse-level actions
        """
        all_fine_actions = []
        fine_actions_per_coarse = []
        fine_plan_length = 0

        fine_state_fluents = copy.deepcopy(self.fine_fluents)

        for i in range(len(coarse_actions)):
            _, course_action_name, *objs = re.split("\(|\)|\,", coarse_actions[i])[:-3]

            # ZOOM
            logging.info(f"Coarse Action: {course_action_name}")
            logging.info(f"Action objects: {objs}")
            actr = ActionInstance(
                self.coarse.get_action_by_name(course_action_name),
                objs,
                coarse_states[i].time_step,
            )
            logging.info("Building zoomed system description...")
            zoomed_fine_res_description = zoom(
                coarse_states[i],
                coarse_states[i + 1],
                actr,
                self.coarse,
                self.fine,
                True,
            )
            fine_prog = zoomed_fine_res_description.to_sparc_program()
            fine_prog.save(
                os.path.join(
                    os.environ["PLANNER_PATH"],
                    "sparc_planning/sparc_files/fine_temp.sp",
                )
            )
            logging.info("Zooming complete")

            # run fine planner
            fine_plan = await plan(
                os.path.join(
                    os.environ["PLANNER_PATH"],
                    "sparc_planning/sparc_files/fine_temp.sp",
                ),
                max_length=fine_plan_length + FINE_ITERATION_LIMIT,
                min_length=fine_plan_length + FINE_ITERATION_MIN,
            )
            # collect results
            fine_states, fine_actions = extract_states_from_answer_set(fine_plan[0])
            logging.info(fine_actions)
            fine_plan_length += len(fine_actions)
            # update fine state history
            # only recieve a partial state back from zoomed fine res system
            # need to update full state, so un-used fluents need to be progressed in time (did not change)
            updated_fluents = fine_states[-1].fluents
            # state strings are timestamped so this needs to be fixed for comparison
            s1flu = [
                remove_chars_from_last_number(f1)
                for f1 in fine_state_fluents
                if f1[0] != "%"
            ]  # if f1[0] != '%' ignores comment lines
            s2flu = [remove_chars_from_last_number(f2) for f2 in updated_fluents]
            s1funcs, s1fun_vals = [], []
            for func in s1flu:
                split = re.split("\(|\)|\,", func)
                s1funcs.append(
                    split[0] + "(" + split[1] + "(" + ",".join(split[2:-3]) + ")"
                )
                s1fun_vals.append(split[-2])
            s2funcs = []
            for func in s2flu:
                split = re.split("\(|\)|\,", func)
                s2funcs.append(
                    split[0] + "(" + split[1] + "(" + ",".join(split[2:-3]) + ")"
                )

            # find fluents which havent been updated in zoomed description
            # functions in s1 but not in s2
            funcs = {*s1funcs}.difference({*s2funcs})
            unchanged_fluents = [
                f"{f},{s1fun_vals[s1funcs.index(f)]},{fine_plan_length})."
                for f in funcs
            ]
            # update fluents at this step
            fine_state_fluents = unchanged_fluents + updated_fluents
            # update fine domain for next planning step
            self.fine.domain_setup = (
                self.fine_statics + fine_state_fluents + self.xml_fine_statics
            )
            self.fine.start_step = fine_plan_length
            all_fine_actions += fine_actions
            fine_actions_per_coarse.append(fine_actions)

        logging.info(f"Coarse Actions: {coarse_actions}")
        logging.info(f"Fine Actions: {all_fine_actions}")
        return all_fine_actions, fine_actions_per_coarse

    def plan(self):
        """
        Planning a high-level actions to assemble beams.
        """
        abstract_states, abstract_actions = asyncio.run(self.run_abstract_plan())
        coarse_states, coarse_actions = asyncio.run(
            self.run_coarse_plan(abstract_states, abstract_actions)
        )
        fine_actions, fine_actions_per_coarse = asyncio.run(
            self.run_fine_plan(coarse_states, coarse_actions)
        )
        return coarse_actions, fine_actions, fine_actions_per_coarse

    def execute(self, coarse_actions, fine_actions, fine_actions_per_coarse):
        """
        Execute low-level actions given a plan
        Args:
            coarse_actions: a list of coarse-leve actions
            fine_actions: a list of fine-level actions
            fine_actions_per_coarse: A nested list containing fine-level actions for each coarse-level action.
        """
        print("Coarse action: ", coarse_actions, "\n")
        print("Fine actions: ", fine_actions)

        start_idx = 0

        fine_actions_per_coarse = self.preprocess_plan(
            coarse_actions, fine_actions_per_coarse
        )

        self.interface.robot.move_to_neutral()
        self.interface.move_to_beam(
            "{}j1".format(self.fixed_beam_id)
        )  # move to the base beam origin
        self.interface.record_fixed_origin()

        for fine_action in fine_actions_per_coarse:
            # insertion_poses = calculate_insertion_poses(self.assembly, fine_action, 0.04, 0.2)
            insertion_poses = calculate_insertion_poses(
                self.assembly, fine_action, 0.055, 0.2
            )
            if len(insertion_poses) != 0:
                self.interface.load_insertion_poses(insertion_poses)

        self.interface.robot.move_to_neutral()
        # for i, coarse_action in enumerate(coarse_actions):
        for i, coarse_action in enumerate(coarse_actions[start_idx:]):
            print("Executing coarse action: ", coarse_action)
            # for action in fine_actions_per_coarse[i]:
            for action in fine_actions_per_coarse[start_idx:][i]:
                group = re.match(self.action_template, action)
                abst_action_group = re.match(self.abst_action_template, group[1])
                abst_action, action_args = abst_action_group[1], abst_action_group[
                    2
                ].split(",")
                order = group[2]

                action_args.remove("rob0")
                print("Executing {}".format(action))
                if abst_action == "move_f":
                    target = action_args[0]
                    if target == "above_input_area":
                        self.interface.move_to_input_area()
                    elif target == "above_intermediate_area":
                        self.interface.move_to_intermediate_area()
                    elif target == "above_assembly_area":
                        self.interface.move_to_assembly_area()
                    elif re.search("(b\d*)i", target) or re.search("(b\d*)t", target):
                        # move to the beam input or target pose
                        self.interface.move_to_beam(target)
                    elif re.search("(b\d*)a", target):
                        # move to the approach pose of the beam for insertion
                        insert_end = get_insert_end(
                            self.assembly, fine_actions_per_coarse[start_idx:][i]
                        )
                        self.interface.move_beam_to_approach_pose(target, insert_end)
                    elif re.search("(p\d*)i", target):
                        # move to the peg input pose
                        self.interface.move_to_peg(target)
                    else:
                        pass
                        # raise ValueError("{}, target: {}".format(abst_action, target))
                elif abst_action == "pick_up_f":  # Pick up action
                    target = action_args[0]
                    if re.search("(b\d*)(l\d*)", target):
                        self.interface.pickup_beam(target)
                    elif re.search("(p\d*)", target):
                        # pick up peg
                        self.interface.pickup_peg(target)
                    else:
                        raise NotImplementedError
                elif abst_action == "putdown_f":
                    # put down a rasped object
                    self.interface.put_down()
                    pass
                elif abst_action == "assemble_f_square":
                    # assemble square action
                    grasped_beam = action_args[0]
                    target_beam = self.get_connected_beam(grasped_beam)
                    self.interface.assemble_beam_square(grasped_beam, target_beam)
                elif abst_action == "fasten":
                    # insert a peg into a hole
                    bj1, bj2, peg = action_args
                    self.interface.insert_peg(bj1, bj2, peg)
                elif abst_action == "assemble_f_cap":
                    # capping with a beam
                    grasped_beam = action_args[0]
                    target_beam = self.get_connected_beam(grasped_beam)
                    self.interface.cap_beams(grasped_beam, target_beam.name)
                elif abst_action == "push":
                    # push a beam
                    target = action_args[0]
                    connected_beams = self.get_connected_beams(target)
                    for beam in connected_beams:
                        if self.assembly.base.name in beam:
                            connected_beam_joint = beam
                        # pre_cap = self.is_capped(fine_actions_per_coarse[:i], target)
                        pre_cap = self.is_capped(
                            fine_actions_per_coarse[: start_idx + i], target
                        )
                    self.interface.push(target, connected_beam_joint, pre_cap)
                else:
                    raise ValueError(abst_action)
        self.interface.put_down()
        self.interface.move_to_intermediate_area()

    def get_connected_beam(self, beam):
        beam_group = re.match("(b\d*)(j.*)", beam)
        for connection in self.assembly.get_beam_connections(beam_group[1]):
            if beam in connection.joint1.name:
                target_beam = connection.joint2
            elif beam in connection.joint2.name:
                target_beam = connection.joint1
        return target_beam

    def get_connected_beams(self, beam):
        beam_group = re.match("(b\d*)", beam)
        connected_beams = []
        for connection in self.assembly.get_beam_connections(beam_group[1]):
            if beam in connection.joint1.name:
                connected_beams.append(connection.joint2.name)
            elif beam in connection.joint2.name:
                connected_beams.append(connection.joint1.name)
        return connected_beams

    def preprocess_plan(self, coarse_actions, fine_actions_per_coarse):
        """
        Preprocess a generated high-level plan. In order to insert a beam to the other beam stably, a robot needs to grasp a link next to a joint that is inserted to the other beam. However, our current planner does not plan to generate such a plan. Thus, we manually update the grasp link position.
        Args:
            coarse_actions: A list of coarse-level actions
            fine_actions_per_coarse: A nested list containing fine-level actions for each coarse-level action.
        """

        pickup_idx = None
        pickup_link = None
        for i, fine_actions in enumerate(fine_actions_per_coarse):
            for j, action in enumerate(fine_actions):
                group = re.match(self.action_template, action)
                abst_action_group = re.match(self.abst_action_template, group[1])
                abst_action, action_args = abst_action_group[1], abst_action_group[
                    2
                ].split(",")
                action_args.remove("rob0")
                if abst_action == "pick_up_f":  # Pick up action
                    target = action_args[0]
                    if re.search("(b\d*)(l\d*)", target):
                        pickup_idx = (i, j)
                        pickup_link = target

                if abst_action == "assemble_f_square":
                    grasped_beam = action_args[0]
                    group = re.match("(b\d*)(j\d*)", grasped_beam)
                    if pickup_idx is None:
                        raise ValueError("pick up action is not found before assembly")
                    fine_actions_per_coarse[pickup_idx[0]][
                        pickup_idx[1]
                    ] = fine_actions_per_coarse[pickup_idx[0]][pickup_idx[1]].replace(
                        pickup_link, "{}l{}".format(group[1], group[2][1:])
                    )
                    pickup_idx = None
                    pickup_link = None
        return fine_actions_per_coarse

    def is_capped(self, fine_actions_per_coarse, target):
        """
        Check if a beam is already capped.
        Args:
            fine_actions_per_coarse: A nested list containing fine-level actions for each coarse-level action.
            target: A target beam
        """
        for fine_actions in fine_actions_per_coarse:
            for action in fine_actions:
                group = re.match(self.action_template, action)
                abst_action_group = re.match(self.abst_action_template, group[1])
                abst_action, action_args = abst_action_group[1], abst_action_group[
                    2
                ].split(",")
                if "cap" in abst_action:
                    beam_id, _ = self.interface.get_beam_joint_id(action_args[1])
                    for connected_beam in self.get_connected_beams(beam_id):
                        if target in connected_beam:
                            return True
        return False
