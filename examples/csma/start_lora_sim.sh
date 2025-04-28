# /bin/bash!

#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=false --interferenceMatrix=goursaud --nDevices=1000 --filePostfix=false" >>./3km_400s_lorawan_adr_without_cad-100.log 2>&1 &
#pid1=$!
#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=true --interferenceMatrix=goursaud --nDevices=1000 --filePostfix=true" >>./3km_400s_lorawan_adr_cad-100.log 2>&1 &
#pid2=$!
#wait $pid1 $pid2

./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=false --interferenceMatrix=goursaud --nDevices=20 --filePostfix=false" >>./3km_400s_lorawan_adr_without_cad-2000.log 2>&1
pid1=$!
./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=true --interferenceMatrix=goursaud --nDevices=20 --filePostfix=true" >>./3km_400s_lorawan_adr_cad-2000.log 2>&1
pid2=$!

#
#
#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=false --interferenceMatrix=goursaud --nDevices=4000 --filePostfix=false" >>./3km_400s_lorawan_adr_without_cad-4000.log 2>&1
#pid1=$!
#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=true --interferenceMatrix=goursaud --nDevices=4000 --filePostfix=true" >>./3km_400s_lorawan_adr_cad-4000.log 2>&1
#pid2=$!
#
#
#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=false --interferenceMatrix=goursaud --nDevices=10000 --filePostfix=false" >>./3km_400s_lorawan_adr_without_cad-10000.log 2>&1
#pid1=$!
#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=true --interferenceMatrix=goursaud --nDevices=10000 --filePostfix=true" >>./3km_400s_lorawan_adr_cad-10000.log 2>&1
#pid2=$!

##./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=false --interferenceMatrix=goursaud --nDevices=750 --filePostfix=false" >>./3km_400s_lorawan_adr_without_cad-750.log 2>&1 &
##pid1=$!
##./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=true --interferenceMatrix=goursaud --nDevices=750 --filePostfix=true" >>./3km_400s_lorawan_adr_cad-750.log 2>&1 &
##pid2=$!
##wait $pid1 $pid2
#
#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=false --interferenceMatrix=goursaud --nDevices=1000 --filePostfix=false" >>./3km_400s_lorawan_adr_without_cad-1000.log 2>&1 &
#pid1=$!
#./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=true --interferenceMatrix=goursaud --nDevices=1000 --filePostfix=true" >>./3km_400s_lorawan_adr_cad-1000.log 2>&1 &
#pid2=$!
#wait $pid1 $pid2