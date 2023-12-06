import './Header.css'

import React, {useState, useEffect} from 'react';

import ROSLIB from "roslib";

import { Box, Stack, Input, Flex, Circle } from '@chakra-ui/react';


function Header({robotBase, robotSerialNumber, status}) {

    return (
        <Stack h='100px' direction='row' spacing='0px' outline='3px solid black'>
            <Flex w='300px' align='center' justifyContent='center' outline='3px solid black'>
                {robotBase}
            </Flex>
            <Flex w='400px' align='center' justifyContent='center' outline='3px solid black'>
                S/N: {robotSerialNumber}
            </Flex>
            <Flex w='224px' align='center' justifyContent='center' outline='3px solid black'>
                Comm Status:
            </Flex>
            <Flex w='100px' align='center' justifyContent='center'  animation='flashing 1750ms linear infinite'>
                {(status) === "CONNECTED" && 
                <Circle w='75px' h='75px' bgColor='green'/>
                }
                {((status) === "CLOSED" || (status) === "ERROR") && 
                <Circle w='75px' h='75px' bgColor='red' />
                }
                {(status) === "Unknown or Connecting" && 
                <Circle w='75px' h='75px' bgColor='yellow' />
                }
            </Flex>
        </Stack>   
    )
}

export default Header