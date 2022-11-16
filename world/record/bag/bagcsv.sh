#$topic1 = "scan"
#$topic2 = "rover_odo"
rostopic echo -b $1.bag -p /scan > ../$1_scan.csv
rostopic echo -b $1.bag -p /rover_odo > ../$1_odo.csv
echo generate $1.csv
