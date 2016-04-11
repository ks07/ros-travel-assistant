#!/usr/bin/env python

import sys

def main(logs):
    cover_passed = 0 # Successes
    cover_others = 0 # Failures or unrecorded
    has_failed = False
    for log in logs:
        with open(log, 'r') as logfile:
            for line in logfile:
                parts = line.strip().split(',')
                # Some assertions tag triggers with data, could cover these?
#                if len(parts) > 2:
#                    # Squash
#                    parts = parts[','.join(a[:-1]),a[-1]]

                if len(parts) < 2:
                    cover_others += 1
                elif parts[-1] == '1':
                    cover_passed += 1
                elif parts[-1] == '0':
                    cover_others += 1
                    has_failed = True
                else:
                    cover_others += 1

    if has_failed:
        print 'WARNING: ASSERTION FAILURES RECORDED'
    print 'Covered, Passed: ', cover_passed
    print 'Covered, Others: ', cover_others

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Usage: assertion_cov_analyse.py <path> [additional paths...]'
    else:
        main(sys.argv[1:])
