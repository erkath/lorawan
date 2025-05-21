# /bin/bash!

device_counts=(50 100 150 200 250 300 350 400 450)

for nDevices in "${device_counts[@]}"
do
    # Запуск без CAD
    ./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=false --interferenceMatrix=goursaud --nDevices=$nDevices --filePostfix=false" >> "./80m_100s_lorawan_adr_without_cad-${nDevices}.log" 2>&1 &
    pid1=$!
    wait $pid1

    # Запуск с CAD
    ./ns3 run "scratch/aloha-throughput.cc --CsmaEnabled=true --interferenceMatrix=goursaud --nDevices=$nDevices --filePostfix=true" >> "./80m_100s_lorawan_adr_cad-${nDevices}.log" 2>&1 &
    pid2=$!
    wait $pid2

done