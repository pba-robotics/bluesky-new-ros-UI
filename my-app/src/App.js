import './App.css';

import React, {useState, useEffect} from 'react';

import ROSLIB from "roslib";

function App() {
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
  const [batteryOutput, setBatteryOutput] = useState("");

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

    });

    ros.on('close', function() {
      setStatus("CLOSED");
    });

    // Create a connection to the rosbridge WebSocket server.
    ros.connect('ws://localhost:9090');


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
      setBatteryOutput(message);
    })
  }, [])


  const checkConnect = () => {
    if (status === "Unknown or Connecting"){
      // console.log(status)
      ros.close()
      setStatus("Error, timeout");
    }
  };

  const connectTimeout = setTimeout(checkConnect, 5000);


  const handleSubmit = (e) => {
    publisher.publish(new ROSLIB.Message({data:messageText}))
    e.preventDefault()
    setMessageText("")
  }

  const handleChange = (e) => {
    setMessageText(e.target.value)
  }

  return (
    <>
    <div className="App" style={{width:1024, height:768}}>
      <header className="App-header">
        <div>
          Connection Status: {status}
        </div>
        <div>
          This is a test to check if ROSLIB is working. <br></br>
          Receieved: {output}
        </div>
        <div>
          Battery Raw Output: {JSON.stringify(batteryOutput)}
        </div>
        <div>
          Battery: {batteryOutput.percentage}
        </div>
        <div>
          <div>
            Write test ROS message below.
          </div>
          <form>
            <input value={messageText} onChange={handleChange} name="ROS-sender"></input>
            <button onClick={handleSubmit}> Submit </button>
          </form>
        </div>
      </header>
    </div>
    </>
  );
}

export default App;
