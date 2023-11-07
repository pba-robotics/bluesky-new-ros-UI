import React from "react"
import { Input } from "@chakra-ui/react";

function Test(props) {
  return (
    <div>
        <div>
            This is a test to check if ROSLIB is working. <br></br>
            Receieved: {props.output}
        </div>
        <div>
            Write test ROS message below.
        </div>
        <form>
            <Input value={props.messageText} onChange={props.onChange} name="ROS-sender"></Input>
            <button onClick={props.onClick}> Submit </button>
        </form>
    </div>
  )
};

export default Test;
