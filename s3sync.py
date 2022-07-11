import subprocess
import os
import json

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


def fileWrite(name, input):
    if(os.stat("jsonTest.json").st_size == 0): #if json file is empty, create new dictionary
        fOut = {name: input}
    else: #if json file has contents: 
        fOut = fileRead() #read existing dictionary from json file
        fOut.update({name: input}) #add new key and contents to file
    with open("jsonTest.json", "r+") as file:
        json.dump(fOut, file)

def fileRead(): #returns contents of the json file as a dictionary
    with open('jsonTest.json', 'r') as file:
        jsonObj = json.load(file)
    return jsonObj

def filePrint():
    with open('jsonTest.json', 'r') as file:
        jsonObj = json.load(file)

    for key in jsonObj:
        print(jsonObj[key])

def fileWipe(): #Delete content of jsonTest.txt file
    file = open("jsonTest.json", "r+")
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
        fileWrite(folder, result)
    print('There are ' + str(folderC) + ' files in this directory')

#result = ""
def sync(folder):
    global result
    path = os.path.join(os.path.expanduser('~'), 'Desktop', 'DATA', 'output',folder)
    arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder), '--dryrun']
    result = subprocess.run(arguments, stdout=subprocess.PIPE).stdout.decode('utf-8')
    syncCheck(result)

    #fileWipe()

#sync('2022-07-06T02:52:41.005402')
#print(outputList)
##fileWipe()
#fileWrite('2022-07-06T02:52:41.005402', result)
#sync('2022-07-06T05:59:35.153863')
#fileWrite('2022-07-06T05:59:35.153863', result)
#filePrint()
##syncAll()

filePrint()


