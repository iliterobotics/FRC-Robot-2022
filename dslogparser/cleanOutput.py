with open('filteredOutput.csv') as output_file:
    with open("final.csv",'w') as final_file:
        for row in output_file:
            csv_row = row.split('BaseAutonController::execute ')[-1]
            csv_row = csv_row.split('<TagVersion>1')[0]
            csv_row = csv_row.strip()
            final_file.write(csv_row)
            final_file.write('\n')