import React from "react"
import { Stack, Box, Flex } from "@chakra-ui/react"

function Dashboard(props) {
  return (
    <Box>
    <Flex h='300px' fontSize='72' justifyContent='center' align='center'>
      <Flex w='300px'>
        Status:
      </Flex>
      <Flex>
        {props.statusOutput.statedetails}
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
