import { HStack, Box, Flex, Button, Center } from "@chakra-ui/react"
import React from "react"
import Plot from 'react-plotly.js'



function Battery(props) {

    //console.log(props.batteryOutputHistory)
    const average = arr => arr.reduce( ( p, c ) => p + c, 0 ) / arr.length;

    var time_data = props.batteryOutputHistory.map((x) => x.header.stamp.secs)
    var battery_data = props.batteryOutputHistory.map((x) => (
        props.selectedBatteryParameter === 'Percentage' ? x.percentage : 
        props.selectedBatteryParameter === 'Current' ? x.current : average(x.cell_temperature)
        ))
    var label = props.selectedBatteryParameter

    //console.log(time_data)

    var data = [
        {
             x: time_data,
             y: battery_data,
             fill: 'tozeroy',
             type: 'scatter',
             name: label
           }
           
    ];

   
    return(
        <Box>
            <HStack marginLeft='auto' marginRight='auto' marginTop='20px' w='1000px' justifyContent='center' spacing='30px'>
                <Button w='320px' h='auto' justifyContent='center' fontSize='42' value='Percentage' _hover={{bg: false}} bgColor={props.selectedBatteryParameter === "Percentage" ? 'aqua' : 'none'} onClick={props.onClick}>
                    Battery %:  <br/>
                    {(props.batteryOutput.percentage*100).toFixed(2)}
                </Button>
                <Button w='320px' h='auto' justifyContent='center' fontSize='42' value='Current' _hover={{bg: false}} bgColor={props.selectedBatteryParameter === "Current" ? 'aqua' : 'none'} onClick={props.onClick}>
                    Current: <br/>
                    {props.batteryOutput.current.toFixed(2)} A
                </Button>
                <Button w='320px' h='auto' justifyContent='center' fontSize='42' value='Temperature' _hover={{bg: false}} bgColor={props.selectedBatteryParameter === "Temperature" ? 'aqua' : 'none'} onClick={props.onClick}>
                    Temperature: <br/>
                    {average(props.batteryOutput.cell_temperature).toFixed(2) + ' \u00B0'}C
                </Button>
            </HStack>
        <Plot
        data={data}
        layout={ {width: 1000, height: 350} }/>
        </Box>
    )
};

export default Battery;
