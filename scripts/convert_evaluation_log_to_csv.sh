echo "Episode,Step,X,Y,Heading,Steering,Speed,Action,Reward,Crashed,On Track,Progress,Waypoint Index,Track_Len,Time,Status,Laps" > $1.csv
egrep "^SIM_TRACE_LOG:" $1 | sed 's/SIM_TRACE_LOG://g' >> $1.csv
