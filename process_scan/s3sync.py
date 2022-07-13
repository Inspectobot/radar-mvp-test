import subprocess
import os
import json
import shutil
import uuid

SCRIPT_DIRECTORY_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__)))
DATA_DIRECTORY_PATH = os.path.abspath(os.path.join(SCRIPT_DIRECTORY_PATH, 'output'))
MESSAGE_TEMPLATE_PATH = os.path.abspath(os.path.join(SCRIPT_DIRECTORY_PATH, 'message.json'))
AWS_SNS_TOPIC_ARN = "arn:aws:sns:us-east-2:731777088509:gpr-data-uploads"

dryRun = False


def getSyncJSONFilePath(name):
    return os.path.join(DATA_DIRECTORY_PATH, name, 'syncData.json')

def syncCheck(name, p, result, sampleCount = 0, lineCount = 0):
    # tbd: given a folder, list all the files on s3 and compare their file sizes with the ones locally
    if(p.returncode == 0):
      messageJSONFilePath = os.path.join(DATA_DIRECTORY_PATH, name, os.path.basename(MESSAGE_TEMPLATE_PATH))

      shutil.copy(MESSAGE_TEMPLATE_PATH, messageJSONFilePath)

      with open(messageJSONFilePath, 'r+') as f:
        message = json.load(f)

        message['id'] = str(uuid.uuid4())
        message['detail-type'] = f"New Scan Upload: s3://gpr-scan-data/{name}/"
        message['region'] = f"Line Count: {lineCount}, Sample Count {sampleCount}"
        f.seek(0)

        json.dump(message, f)

      subprocess.run(['aws', 'sns', 'publish', '--topic-arn', f"{AWS_SNS_TOPIC_ARN}", '--message', f'file://{messageJSONFilePath}']);

    else:
      print(f"sync non-zero return code: {p.returncode}")
    
    if(result == ""): #if no result, print 'no change'
            print('no changes')
    else: #else, we have a result and should print it
        print(result)

#syncCheck('2022-07-13T12:31:20.768071', True, '')



def fileWrite(name, input):
    if(os.path.exists(getSyncJSONFilePath(name)) != True or os.stat(getSyncJSONFilePath(name)).st_size == 0): #if json file is empty, create new dictionary
        files = input.splitlines()
        files[:] = [x for x in files if x]
        fOut = {name: files}

    else: #if json file has contents: 
        fOut = fileRead(name) #read existing dictionary from json file
        files = input.splitlines()
        files[:] = [x for x in files if x]
        fOut = {name: files}

    with open(getSyncJSONFilePath(name), "w") as file:
        json.dump(fOut, file)

def fileWriteDict(input):
    if(os.path.exists(getSyncJSONFilePath(name)) != True or os.stat(getSyncJSONFilePath(name)).st_size == 0): #if json file is empty, create new dictionary
        fOut = input
    else: #if json file has contents: 
        fOut = fileRead(name) #read existing dictionary from json file
        files = input.splitlines()
        files[:] = [x for x in files if x]
        fOut = {name: files}
        
    with open(getSyncJSONFilePath(name), "w") as file:
        json.dump(fOut, file)

def fileRead(name): #returns contents of the json file as a dictionary
    with open(getSyncJSONFilePath(name), 'r') as file:
        jsonObj = json.load(file)
    return jsonObj

def filePrint(name):
    with open(getSyncJSONFilePath(name), 'r') as file:
        jsonObj = json.load(file) #load json dict from given file
    for key in jsonObj:
        print(jsonObj[key]) #print the value for each and every key

def recordPrint(folder): #returns sync records for particular folder
    with open(getSyncJSONFilePath(folder), 'r') as file:
        jsonObj = json.load(file) #load json dict from given file
    print(jsonObj[folder]) #print sync results of given folder (folder name used as key)

result = ""
def getResult():
    global result
    return result

def syncAll(): #version of sync() that syncs every file in the directory
    global folderC, result
    allResults = result
    folderName = ""
    for folder in outputList:
        folderName = folder
        path = os.path.join(DATA_DIRECTORY_PATH, folder) #dynamically create path to current folder
        arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder)] #build command to sync file
        if dryRun:
            arguments.append('--dryrun')

        p = subprocess.run(arguments, stdout=subprocess.PIPE)
        result = p.stdout.decode('utf-8')
        allResults += result
        syncCheck(folder, p, result) #if we have results, print them
        folderC += 1
        #fileWrite(folder, result)
    print('There are ' + str(folderC) + ' files in this directory')
    result = allResults

def syncAllTest(): #an iteration-limited version of syncAll to save time testing
    global folderC, result
    allResults = result
    folderName =""
    for folder in outputList[:9]:  #slice can be changed to test a smaller or greater number of iterations
        folderName = folder
        path = os.path.join(DATA_DIRECTORY_PATH, folder) #dynamically create path to current folder
        arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder)] #build command to sync file
        if dryRun:
            arguments.append('--dryrun')
        
        p = subprocess.run(arguments, stdout=subprocess.PIPE)
        result = p.stdout.decode('utf-8')
        allResults += result
        syncCheck(folder, p, result) #if we have results, print them
        folderC += 1
    print('You iterated through ' + str(folderC) + ' files in this directory')
    result = allResults

def sync(folder): #sync single given folder to s3
    global result
    path = os.path.join(DATA_DIRECTORY_PATH, folder)
    arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder)]
    if dryRun:
        arguments.append('--dryrun')
    
    p = subprocess.run(arguments, stdout=subprocess.PIPE)
    result = p.stdout.decode('utf-8')
    syncCheck(folder, p, result)
    
def pathSync(path, folder, sweepCount, lineCount): #sync single folder from given path to s3
    global result
    arguments =  ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder)]
    if dryRun:
        arguments.append('--dryrun')
    
    p = subprocess.run(arguments, stdout=subprocess.PIPE)
    result = p.stdout.decode('utf-8')
    syncCheck(folder, p, result, sweepCount, lineCount)
