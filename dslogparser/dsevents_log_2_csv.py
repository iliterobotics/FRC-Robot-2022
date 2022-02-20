import csv
import logging
import logging.config

import sys, getopt
from dslogparser import DSEventParser

logging.config.fileConfig(fname='logger.conf', disable_existing_loggers=False)

logger = logging.getLogger(__name__)


def main(argv):
    inputfile = None
    outputfile = None

    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
        logger.debug(f'Opts are: {opts}')

        for opt, arg in opts:
            logger.debug(f'Working on option: {opt} val={arg}')

            if opt == '-h':
                print('dsevents_log_2_csv.py -i <inputfile> -o <outputfile>')
                logger.debug("Help option selected")
                exit(0)
            elif opt in ('-i','--ifile'):
                logger.debug(f'input option selected, setting input file to: {arg}')
                inputfile = arg
            elif opt in ('-o','--ofile'):
                logger.debug(f'output option selected, setting output file to: {arg}')
                outputfile = arg
        
        if inputfile and outputfile:
            parser = DSEventParser(inputfile)
            records = parser.read_records()

            with open(outputfile,'w',newline='') as csv_out:
                csv_writer = csv.writer(csv_out)
    
                for r in records:
                    csv_writer.writerow([r['time'],r['message']])
        else:
            logger.warning(f'One of the values: input={inputfile}, output={outputfile} is None. Unable to continue')
    
    except getopt.GetoptError:
        logger.exception('Error parsing the options')
        
        

if __name__ =='__main__':
    main(sys.argv[1:])