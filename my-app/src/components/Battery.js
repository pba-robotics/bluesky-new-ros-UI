import { HStack, Box, Flex } from "@chakra-ui/react"
import React from "react"
import Plot from 'react-plotly.js'



function Battery(props) {

    console.log(props.batteryOutputHistory)

    var time_data = props.batteryOutputHistory.map((x) => x.header.stamp.secs)
    var percentage_data = props.batteryOutputHistory.map((x) => x.current)

    console.log(time_data)

    var data = [
        {
             x: time_data,
             y: percentage_data,
             fill: 'tozeroy',
             type: 'scatter',
             name: 'Battery'
           }
           
    ];

    const average = arr => arr.reduce( ( p, c ) => p + c, 0 ) / arr.length;
    return(
        <Box>
            <HStack w='1024px' justifyContent='center'>
                <Flex w='341px' justifyContent='center'>
                    Battery %:  <br/>
                    {(props.batteryOutput.percentage*100).toFixed(2)}
                </Flex>
                <Flex w='341px' justifyContent='center'>
                    Current: <br/>
                    {props.batteryOutput.current.toFixed(2)} A
                </Flex>
                <Flex w='341px' justifyContent='center'>
                    Temperature: <br/>
                    {average(props.batteryOutput.cell_temperature).toFixed(2) + ' \u00B0'}C
                </Flex>
            </HStack>
        <Plot
        data={data}
        layout={ {width: 1000, height: 350} }/>
        </Box>
    )
};

export default Battery;
