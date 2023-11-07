import './App.css';

import React, {useState, useEffect} from 'react';

import ROSLIB from "roslib";

import { Box, Stack, Input, Flex } from '@chakra-ui/react';

import Header from './components/Header'
import Test from './components/Test'; 
import Battery from './components/Battery';
import Buttons from './components/Buttons';
import Navigation from './components/Navigation';
import Settings from './components/Settings';
import Dashboard from './components/Dashboard';

function App() {
  const [selectedView, setSelectedView] = useState("Dashboard");
  const [output, setOutput] = useState("No data yet");
  const [status, setStatus] = useState("Unknown or Connecting");
  const [messageText, setMessageText] = useState("");
  const [ros, setRos] = useState(new ROSLIB.Ros())
  const [listener, setListener] = useState(new ROSLIB.Topic({
    ros : ros,
    name : '/listener',
    messageType : 'std_msgs/String'
  }));
  const [publisher, setPublisher] = useState(new ROSLIB.Topic({
    ros: ros,
    name : '/listener',
    messageType : 'std_msgs/String'
  }))
  const [battery_listener, setBattery_Listener] = useState(new ROSLIB.Topic({
    ros : ros,
    name : '/bms_handler/veh_battery',
    messageType : 'sensor_msgs/BatteryState'
  }))
  const [batteryOutput, setBatteryOutput] = useState(new ROSLIB.Message({
    header:{ 
      seq: 0,
      stamp:{
        secs: 0,
        nsecs: 0,
        },
      frame_id: ''
      },
    voltage: NaN,
    temperature: 0.0,
    current: NaN,
    charge: NaN,
    capacity: 38.16999816894531,
    design_capacity: 40.0,
    percentage: NaN,
    power_supply_status: 2,
    power_supply_health: 0,
    power_supply_technology: 4,
    present: "True",
    cell_voltage: [3.8289999961853027, 3.8320000171661377, 3.8320000171661377, 3.8299999237060547, 3.8299999237060547, 3.8299999237060547, 3.811000108718872],
    cell_temperature: [25.45000648498535, 25.649988174438477, 27.149988174438477, 30.149988174438477],
    location: "AGV",
    serial_number: ''

  }));
  const [batteryOutputHistory, setBatteryOutputHistory] = useState([]);
  const [selectedBatteryParameter, setSelectedBatteryParameter] = useState('Percentage')
  const [statusOutput, setStatusOutput] = useState(new ROSLIB.Message({
    battpercentage: NaN,
    battremaining: NaN,
    localizationscore: NaN,
    ipaddress: "Uninitialised",
    mapid: "Uninitialised",
    x: 0.0,
    y: 0.0,
    z: 0.0,
    theta: 0.0,
    state: '',
    statedetails: "Uninitialised"
  }))
  const [status_listener, setStatus_Listener] = useState(new ROSLIB.Topic({
    ros : ros,
    name : '/navibee/robotstatus',
    messageType : 'navibee_msgs/RobotStatus'
  }))

  // only for testing purposes
  // const [battery_publisher, setBattery_Publisher] = useState(new ROSLIB.Topic({
  //   ros : ros,
  //   name : '/bms_handler/veh_battery',
  //   messageType : 'sensor_msgs/BatteryState'
  // }))

  // useEffect so this only happens once on initial render
  useEffect(() => {
    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error) {
      setStatus("ERROR");
      console.log(error);
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function() {
      setStatus("CONNECTED");
      clearTimeout(connectTimeout);
      publisher.advertise()

      // battery_publisher.advertise()

    });

    ros.on('close', function() {
      setStatus("CLOSED");

      // TODO: Some form of auto retry if closed
    });

    // Create a connection to the rosbridge WebSocket server.
    ros.connect('ws://192.168.0.10:9090'); // For GR200
    //ros.connect('ws://0.0.0.0:9090');


    // Then we add a callback to be called every time a message is published on this topic.
    listener.subscribe(function(message) {
      console.log("heard message", message.data);
      setOutput(message.data)
      // If desired, we can unsubscribe from the topic as well.
      // listener.unsubscribe();
    });
    publisher.publish(new ROSLIB.Message({data:"test"}))


    battery_listener.subscribe(function(message) {
      console.log("battery message", message);
      setBatteryOutputHistory(batteryOutputHistory => batteryOutputHistory.concat(message))
      // setBatteryOutputHistory(
      //   [
      //     ...batteryOutputHistory,
      //     message
      //   ]
      // )
      setBatteryOutput(message);
    })

    status_listener.subscribe(function(message) {
      setStatusOutput(message);
    })
  }, [])


  const checkConnect = () => {
    if (status === "Unknown or Connecting"){
      // console.log(status)
      ros.close()
      setStatus("Error, timeout");
    }
  };
  
  // TODO: Check if new messages are published

  const connectTimeout = setTimeout(checkConnect, 5000);


  const handleSubmit = (e) => {
    publisher.publish(new ROSLIB.Message({data:messageText}))
    e.preventDefault()
    setMessageText("")
    // battery_publisher.publish(new ROSLIB.Message())
  }

  const handleChange = (e) => {
    setMessageText(e.target.value)
  }

  const handleButtonClick = (e) => {
    console.log(e)
    setSelectedView(e.target.value)
  }

  const handleBatteryButtonClick = (e) => {
    console.log(e)
    setSelectedBatteryParameter(e.target.value)
  }

  return (
    <>
    <Flex className="App" w='1024px' h='768px' fontSize='36' border='4px' flexDirection='column'>
      <Header status={status}/>
      {selectedView === "Dashboard" && <Dashboard batteryOutput={batteryOutput} statusOutput={statusOutput} />}
      {selectedView === "Navigation" && <Navigation/>}
      {selectedView === "Battery" && 
      <Battery batteryOutput={batteryOutput} batteryOutputHistory={batteryOutputHistory} 
      selectedBatteryParameter={selectedBatteryParameter} onClick={handleBatteryButtonClick}/>}
      {selectedView === "Settings" && <Settings/>}
      <Buttons selectedView={selectedView} onClick={handleButtonClick} />
    </Flex>
    <Test output={output} messageText={messageText} onChange={handleChange} onClick={handleSubmit}/>
    <div>
        Battery Raw Output: {JSON.stringify(batteryOutput)}
    </div>
    </>
  );
}

export default App;
