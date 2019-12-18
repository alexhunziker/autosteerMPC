import re

content = open('log_7.log', 'r').read()

# Get lat, lon for plotting
#regex = r'GPS State is {\'lat\': (.*?), \'lon\': (.*?),'
#regex = r'INFO: Throttle (.*?) Breaks (.*?) Steering (.*?)\n'
#print(regex)
#results = re.findall( regex , content)
#results = list(set(results))
#for r in results:
    #print(r[2], r[0], r[1], sep="\t")
#    print(r[0], r[1], sep="\t")

#regex = r'DEBUG: current state is:(.*?)\n'
#regex = r'DEBUG: target state is:(.*?)\n'
#regex = r'INFO: best controll:(.*?)\n'
#regex = r'INFO: Lidar and ultrasonic dist (.*?)\n'
regex = r'INFO: System time is 2019-12-(.*?)\n'
results = re.findall( regex , content)
for r in results:
    print("\t".join(r.split()))
