import subprocess
import os

#arguments = ['aws', 's3', 'sync', '/media/brad/DATA/output/2022-07-06T02:36:57.084556', 's3://gpr-scan-data/testFolder3', '--dryrun']
#result = subprocess.run(arguments, stdout=subprocess.PIPE).stdout.decode('utf-8')

#if(result == ""):
#    print('no changes')
#else:
#    print(result)

outputList = os.listdir('/home/brad/Desktop/DATA/output')
for folder in outputList:
    path = os.path.join(os.path.expanduser('~'), 'Desktop', 'DATA', 'Output',folder)

def syncCheck(result):
    if(result == ""):
            print('no changes')
    else:
        print(result)

outputList = os.listdir('/home/brad/Desktop/DATA/output')
folderC = 0

def syncAll():
    for folder in outputList:
    #print(folder)
        path = os.path.join(os.path.expanduser('~'), 'Desktop', 'DATA', 'output',folder)
        #if folderC < 1:
        arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder), '--dryrun']
        result = subprocess.run(arguments, stdout=subprocess.PIPE).stdout.decode('utf-8')
        syncCheck(result)

        folderC += 1

def sync(folder):
    path = os.path.join(os.path.expanduser('~'), 'Desktop', 'DATA', 'output',folder)
    arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder), '--dryrun']
    arguments = ['aws', 's3', 'sync', path, ('s3://gpr-scan-data/' + folder), '--dryrun']
    result = subprocess.run(arguments, stdout=subprocess.PIPE).stdout.decode('utf-8')
    syncCheck(result)

sync('2022-07-06T02:36:57.084556')


print(folderC)
