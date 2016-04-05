#!/usr/bin/env python

import sys
import pickle

def main(datapath):
    with open(datapath, 'r') as datafile:
        covdata = pickle.load(datafile)

    total_outcomes = 0
    covered_outcomes = 0

    for state, outcomes in covdata.iteritems():
        total_outcomes += len(outcomes)
        for outcome, covered in outcomes.iteritems():
            if covered:
                covered_outcomes += 1

    print 'Structural Coverage of SMACH State Machine:'
    print 'Outcomes Covered: ', covered_outcomes
    print 'Outcomes Missed: ', total_outcomes - covered_outcomes
    print 'Coverage Achieved: ', float(covered_outcomes) / total_outcomes

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Usage: smach_coverage_report.py <path>'
    main(sys.argv[1])
