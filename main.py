import os, sys
import numpy as np

from core.agent import Agent
import argparse
# from sparc_planning.src.example_domains.example_latest import generate_domain_setup
# from sparc_planning.src.example_domains.example_easy_1 import generate_domain_setup
from sparc_planning.src.example_domains.example_easy_2 import generate_domain_setup

if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--assembly_xml", type=str, default=None)
    # args = parser.parse_args()

    agent = Agent(beam_xml=os.path.join(os.environ['PLANNER_PATH'], 'example_beamset_latest.xml'),
                  assembly_xml=os.path.join(os.environ['PLANNER_PATH'], 'assembly_easy_2.xml'),
                  domain_generator=generate_domain_setup)
    coarse_actions, fine_actions, fine_actions_per_coarse = agent.plan()
    print('Execute...')
    agent.execute(coarse_actions, fine_actions, fine_actions_per_coarse)
