import os, sys
import numpy as np
import pickle

from core.agent import Agent
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run the RAMP simulator with the planner for a given assembly.')
    parser.add_argument("-a", "--assembly", type=str,
                        default='assembly_easy_3.xml', help='The assembly XML file to use.')

    args = parser.parse_args()
    if args.assembly == "assembly_easy_1.xml":
        from sparc_planning.src.example_domains.example_easy_1 import generate_domain_setup
    elif args.assembly == "assembly_easy_2.xml":
        from sparc_planning.src.example_domains.example_easy_2 import generate_domain_setup
    elif args.assembly == "assembly_easy_3.xml":
        from sparc_planning.src.example_domains.example_easy_3 import generate_domain_setup

    agent = Agent(beam_xml=os.path.join(os.environ['PLANNER_PATH'], 'example_beamset_latest.xml'),
                  assembly_xml=os.path.join(os.environ['PLANNER_PATH'], args.assembly),
                  domain_generator=generate_domain_setup)
    with open('coarse_actions_latest.pkl', 'rb') as f:
        coarse_actions = pickle.load(f)
    with open('fine_actions_latest.pkl', 'rb') as f:
        fine_actions = pickle.load(f)
    with open('fine_actions_per_coarse_latest.pkl', 'rb') as f:
        fine_actions_per_coarse = pickle.load(f)

    # coarse_actions, fine_actions, fine_actions_per_coarse = agent.plan()
    print('Execute...')
    agent.execute(coarse_actions, fine_actions, fine_actions_per_coarse)
