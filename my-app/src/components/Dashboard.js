import React from "react"
import { Stack, Box, Flex } from "@chakra-ui/react"
import ScaleText from "react-scale-text";

function Dashboard(props) {
  return (
    <Box>
    <Flex h='300px' w='1016px' fontSize='72' justifyContent='center' align='center' >
      <Flex w='300px'>
        Status:
      </Flex>
      <Flex h='300px' w='600px' justifyContent='center'>
        <Flex marginTop='auto' marginBottom='auto'>
          <ScaleText>
          {props.statusOutput.statedetails}
          </ScaleText>
        </Flex>
      </Flex>
    </Flex>
    <Stack direction='row' justifyContent='center' fontSize='72' outline='3px solid black'>
        <Flex w='441px' justifyContent='center'>
            Battery %:  <br/>
            {(props.batteryOutput.percentage*100).toFixed(2)}
        </Flex>
        <Flex w='441px' justifyContent='center'>
            Current: <br/>
            {props.batteryOutput.current.toFixed(2)} A
        </Flex>
    </Stack>
    </Box>
  )
};

export default Dashboard;
