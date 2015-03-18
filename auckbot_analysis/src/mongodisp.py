import pymongo as m
import datetime
# import matplotlib as mp
import matplotlib.pyplot as plt
import numpy as np

# returns a somewhat descriptive string for a certain trip document
def toString(document):
	length = "%05.2f" % document["Length [m]"]
	setup = cleanSetup(str(document["Planner Setup"]))
	return ("t:" + str(document["start_time"]) 
	  + " || l:" + length 
#	  + " || s:" + setup
#	  + " || g:" + str(document["Goal [m, m, rad]"])
	  + " || c: %06.2f" % document["Current [As]"]
    )

def cleanSetup(setup):
	setup = setup.replace("\n", "")
	setup = setup.replace("MB_USE_GRID_PATH: ", "")
	setup = setup.replace("base_global_planner: ", "")
	setup = setup.replace("base_local_planner: ", "")
	return setup

def plotRoute(route):
	routearr = np.array(route)
	rax = route_fig.add_subplot(111, aspect='equal')
	rax.plot(routearr[:,0], routearr[:,1])

def plotStats(datdict):
	params = ["Current [As]", "Length [m]", "Rotation [rad]", "Duration [ms]"]
	sax = []

	for i in range(len(params)):
		data = []
		names = []
		keyz = datdict.keys()
		keyz.sort()
		for k in keyz:
			l = datdict[k]
			rd = [] # route data
			for r in l:
				if params[i] is "Duration [ms]":
					vals = (np.array(r[params[i]])/1000).tolist()
					rd.append(vals)
				else:
					rd.append(r[params[i]])
			data.append(rd)
			names.append(k)

		sax.append(stat_fig.add_subplot(141 + i))
		rang = range(1, len(names) + 1)
		plt.xticks(rang, names, rotation=90)
		if params[i] is "Duration [ms]":
			plt.title("Duration [s]")
		elif params[i] is "Current [As]":
			plt.title("Charge [As]")
		else:
			plt.title(params[i])
		sax[i].boxplot(data)
		print params[i]
		for j in range(len(keyz)):
			print np.mean(data[j])
# --------------------------------------------------------------------------------------------------------

client = m.MongoClient('mongodb://localhost:27017/')
db = client.nav_analysis
collection = db['trips']

datgroupnr = 4

if datgroupnr is 1:
	datagroups = {
		"DWA": 
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 6)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "dwa_local_planner/DWAPlannerROS"}}
			]}, 
	#	"EDWA v0.6": # hint: 0.6 wg 6th feb
	#		{"$and": [
	#			{"start_time": {"$gt": datetime.datetime(2015, 2, 6)}},
	#			{"start_time": {"$lt": datetime.datetime(2015, 2, 7)}},
	#			{"Length [m]": {"$gt": 5}},
	#			{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}}
	#		]}, 
	#	"EDWA v0.8": # hint: 0.8 wg 8th feb
	#		{"$and": [
	#			{"start_time": {"$gt": datetime.datetime(2015, 2, 8)}},
	#			{"start_time": {"$lt": datetime.datetime(2015, 2, 8, 12)}},
	#			{"Length [m]": {"$gt": 5}},
	#			{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}}
	#		]}, 
		"EDWA v0.81": # with new model params and fabs for vel
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 8, 12)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 8, 12, 50)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}}
			]}, 
		"EDWA v0.82": # with path distance bias = 64
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 8, 12, 50)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 8, 14, 05)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},  
		"EDWA v0.83": # increased roational parameters
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 8, 14, 05)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 9)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]}
	}
if datgroupnr is 2:
	datagroups = {
		"to right" :
	    {"Goal [m, m, rad]": [0.5, 3.0, 1.5707963705062866]},
	  "to left" :
	    {"Goal [m, m, rad]": [2, -4, 0]}
	}
if datgroupnr is 3:
	datagroups = {
		"DWA":  #new planner config 
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 9)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 10)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "dwa_local_planner/DWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},
		"EDWA v0.9": #new planner config 
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 9)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 9, 18, 30)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},
		"EDWA v0.91": #fabs for all accelerations
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 9, 18, 30)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 9, 18, 55)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},
		"EDWA v0.92": # valuing traj cost more
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 9, 18, 55)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 10)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},
		"EDWA v0.99": # new map
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 10)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 10, 11)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},
		"EDWA v0.991": # caring even more for trajectory
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 10, 11)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 10, 11, 20)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]}
	}
if datgroupnr is 4:
	datagroups = {
		# "DWA":  #new goals / plan
		# 	{"$and": [
		# 		{"start_time": {"$gt": datetime.datetime(2015, 2, 10)}},
		# 		{"start_time": {"$lt": datetime.datetime(2015, 2, 10, 11, 50)}},
		# 		{"Length [m]": {"$gt": 5}},
		# 		{"Planner Setup": {"$regex" : "dwa_local_planner/DWAPlannerROS"}},
		#     {"Success": {"$gt": 0}}
		# 	]},
		"EDWA": # lower traj_scale, lower self_scale
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 10, 11, 20)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 11)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "edwa_local_planner/EDWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},
		"DWA":  #new goals / plan
			{"$and": [
				{"start_time": {"$gt": datetime.datetime(2015, 2, 10, 11, 50)}},
				{"start_time": {"$lt": datetime.datetime(2015, 2, 11)}},
				{"Length [m]": {"$gt": 5}},
				{"Planner Setup": {"$regex" : "dwa_local_planner/DWAPlannerROS"}},
		    {"Success": {"$gt": 0}}
			]},
  }

# plot
route_fig = plt.figure()
stat_fig = plt.figure()

groups = {}

keyz = datagroups.keys()
keyz.sort()
print keyz
for gr in keyz:
	routes = []
	print gr + " ========================================"
	for trip in collection.find(datagroups[gr]).sort("start_time"):
		print toString(trip)
		routes.append(trip)
	groups[gr] = routes
	print " count: " + str(collection.find(datagroups[gr]).count())

print "==============================================="


plotStats(groups)

# route_fig.show()
stat_fig.show()

input("Press Enter to continue...")

