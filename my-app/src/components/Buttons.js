import { ButtonGroup, Button } from "@chakra-ui/react";
import React from "react"

function Buttons(props) {
  return (
    <ButtonGroup h='130px' justifyContent='center' alignItems='center' marginTop='auto'>
      {/* <Button w='230px' h='100px' _hover={{bg: false}} variant={props.selectedView === "Dashboard" ? 'solid' : 'outline'} colorScheme='cyan' onClick={props.onClick} value="Dashboard">
       */}
      <Button w='230px' h='100px' _hover={{bg: false}} bgColor={props.selectedView === "Dashboard" ? 'aqua' : 'none'} onClick={props.onClick} value="Dashboard">
        Dashboard
      </Button>
      <Button w='230px' h='100px' _hover={{bg: false}} bgColor={props.selectedView === "Navigation" ? 'aqua' : 'none'} onClick={props.onClick} value="Navigation"> 
        Navigation
      </Button>
      <Button w='230px' h='100px' _hover={{bg: false}} bgColor={props.selectedView === "Battery" ? 'aqua' : 'none'} onClick={props.onClick} value="Battery">
        Battery
      </Button>
      <Button w='230px' h='100px' _hover={{bg: false}} bgColor={props.selectedView === "Settings" ? 'aqua' : 'none'} onClick={props.onClick} value="Settings">
        Settings
      </Button>
    </ButtonGroup>
  )
};

export default Buttons;
