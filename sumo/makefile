BOLD_BLACK='\033[1;32m'       # Black
END_COLOR='\033[0m'

SIM_TIME=1200

# Generate 100 passenger cars with speed between 10-20 m/s
CAR_PARAMS=-e $(SIM_TIME) --flows 150 --trip-attributes="type=\"carType\" color=\"255,255,255\""

all: buildmap trips trace

buildmap:
	netconvert --osm-files map.osm -o map.net.xml
	polyconvert --net-file map.net.xml --osm-files map.osm -o map.poly.xml

trace:
	@echo "\033[34m Creating a SUMO trace \033[0m"
	sumo -c sim.sumocfg --fcd-output sumoTrace.xml
	@echo "\033[92m Exporting to ns-2 trace \033[0m"
	python3 traceExporter.py --fcd-input sumoTrace.xml --ns2mobility-output ns2mobility.tcl

trips:
	@echo "\033[92m Making Trips \033[0m"
	python3 randomTrips.py -n map.net.xml -r car_routes.rou.xml -o car_trips.xml --vtype-output car_vtypes.xml $(CAR_PARAMS)
	@echo "\033[92m Adding vehicle type for speed constraints \033[0m"
	echo '<vType id="carType" vClass="passenger" minSpeed="10" maxSpeed="20"/>' > car_vtypes.xml
	@echo "\033[92m Creating unique IDs in route files \033[0m"
	sed -i "s/vehicle id=\"/vehicle id=\"car/g" car_routes.rou.xml

sim:
	sumo-gui -c sim.sumocfg

clean:
	rm -f sumoTrace.xml ns2mobility.tcl
	rm -f *.rou.xml *.rou.alt.xml *trips.xml *vtypes.xml
	rm -f map.net.xml map.poly.xml

