# Original Source
The original source was from the following repo: https://github.com/ligerbots/dslogparser which uses the MIT License as defined at: https://github.com/ligerbots/dslogparser/blob/master/LICENSE.txt. Originally the plan was to use this project through Pypi dependencies. However since this version does not handle version 4 of the logs, we were unable to use this project through Pypi

## Quick hack version 
A quick hack version was developed at: https://github.com/caseyjbrotherton/dslogparser/tree/quickhack which addresses the version 4 issue. Since this version has not been deployed to Pypi, the only way to use this code was to include in this project. Once version 4 parsing is deployed to Pypi, then the checked in versions of the code will be dropped.


# How to run

## Setup
1. Setup a virtual environment: https://docs.python.org/3/tutorial/venv.html 
2. Activate the virtual envrionement
3. Install all dependencies: pip install -r requirements.txt

## Running the code
```
python parse_files <full path to .dsevents file>