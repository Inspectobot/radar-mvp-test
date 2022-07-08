import subprocess
result = subprocess.run(['aws', 's3', 'sync', '/media/brad/DATA/output/2022-07-06T02:36:57.084556', 's3://gpr-scan-data/testFolder3', '--dryrun'], stdout=subprocess.PIPE)
result = result.stdout.decode('utf-8')

if(result == ""):
    print('no changes')
else:
    print(result)
