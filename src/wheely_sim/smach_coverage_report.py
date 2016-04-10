#!/usr/bin/env python

import sys
import pickle

def combine_coverage(records):
    dest = records[0]
    for covdata in records[1:]:
        for state, outcomes in covdata.iteritems():
            for outcome, covered in outcomes.iteritems():
                dest[state][outcome] |= covered
    return dest

def include_point(state,outcome):
    if outcome == 'preempted':
        return False
    elif state == 'WAITING' and outcome == 'shutdown':
        # FIXME: This should get covered but isn't
        return False
    return True

def main(datapaths):
    covdatas = []
    for datapath in datapaths:
        with open(datapath, 'r') as datafile:
            covdatas.append(pickle.load(datafile))
    covdata = combine_coverage(covdatas)

    total_outcomes = 0
    covered_outcomes = 0

    for state, outcomes in covdata.iteritems():
        for outcome, covered in outcomes.iteritems():
            if include_point(state,outcome):
                total_outcomes += 1
                if covered:
                    covered_outcomes += 1

    print 'Structural Coverage of SMACH State Machine:'
    print 'Outcomes Covered: ', covered_outcomes
    print 'Outcomes Missed: ', total_outcomes - covered_outcomes
    print 'Coverage Achieved: ', float(covered_outcomes) / total_outcomes

    print '\nCoverage Holes:'
    for state, outcomes in covdata.iteritems():
        new = True
        for outcome, covered in outcomes.iteritems():
            if not covered and include_point(state,outcome):
                if new:
                    print state
                    new = False
                print '  ', outcome

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Usage: smach_coverage_report.py <path> [additional paths...]'
    main(sys.argv[1:])
