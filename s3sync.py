import subprocess
import os

#arguments = ['aws', 's3', 'sync', '/media/brad/DATA/output/2022-07-06T02:36:57.084556', 's3://gpr-scan-data/testFolder3', '--dryrun']
#result = subprocess.run(arguments, stdout=subprocess.PIPE).stdout.decode('utf-8')

#if(result == ""):
#    print('no changes')
#else:
#    print(result)

outputList = os.listdir('/home/brad/Desktop/DATA/output')

def syncCheck(result):
    if(result == ""):
            print('no changes')
    else:
        print(result)

def fileWrite(input): #copy over text from input to jsonTest.txt file
    file = open("jsonTest.txt", "r+")
    for l in input:
        file.write(l)
    file.close()

def fileRead(): #read content of jsonTest.txt by line
    file = open("jsonTest.txt", "r")
    for l in file:
        print(l)
    if(os.stat("jsonTest.txt").st_size == 0): #check if file is empty, useful for fileWipe()
        print("File is empty")
    file.close()

def fileWipe(): #Delete content of jsonTest.txt file
    file = open("jsonTest.txt", "r+")
    file.truncate()


outputList = os.listdir('/home/brad/Desktop/DATA/output')
global folderC
folderC = 0

def syncAll():
    for folder in outputList:
        global folderC
        path = os.path.join(os.path.expanduser('~'), 'Desktop', 'DATA', 'output',folder) #dynamically create path to current folder
        #if folderC < 1:
        arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder), '--dryrun'] #build command to sync file
        result = subprocess.run(arguments, stdout=subprocess.PIPE).stdout.decode('utf-8') #sync file with prebuilt command and save results
        syncCheck(result) #if we have results, print them

        folderC += 1
    print('There are ' + str(folderC) + ' files in this directory')

def sync(folder):
    path = os.path.join(os.path.expanduser('~'), 'Desktop', 'DATA', 'output',folder)
    arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder), '--dryrun']
    result = subprocess.run(arguments, stdout=subprocess.PIPE).stdout.decode('utf-8')
    syncCheck(result)
    fileWrite(result)

sync('2022-07-06T02:52:41.005402')



