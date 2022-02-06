import csv
import sys, getopt
from dslogparser import DSEventParser

def main(argv):
    inputfile = None
    outputfile = None

    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
        print(opts)

        for opt, arg in opts:
            print(opt,arg)

            if opt == '-h':
                print('dsevents_log_2_csv.py -i <inputfile> -o <outputfile>')
            elif opt in ('-i','--ifile'):
                inputfile = arg
            elif opt in ('-o','--ofile'):
                outputfile = arg
        
        if inputfile and outputfile:
            parser = DSEventParser(inputfile)
            records = parser.read_records()

            with open(outputfile,'w',newline='') as csv_out:
                csv_writer = csv.writer(csv_out)
    
                for r in records:
                    csv_writer.writerow([r['time'],r['message']])
        else:
            print('need an input and an output')
    
    except getopt.GetoptError:
        print('parse_files -i <input_file> -o <outputfile>')
        
        

if __name__ =='__main__':
    main(sys.argv[1:])