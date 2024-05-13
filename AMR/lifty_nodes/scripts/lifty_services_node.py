#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import Servo
from gpiozero import TonalBuzzer
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from luma.core.render import canvas
from PIL import Image
from time import sleep
from fyp_interfaces.srv._fork import Fork
from fyp_interfaces.srv._buzzer import Buzzer
from fyp_interfaces.srv._oled import Oled

class LiftyServices(Node):
        def __init__(self):
                super().__init__('lifty_services')
                self.forklift_service = self.create_service(Fork, 'lifty_servic>
                self.buzzer_service = self.create_service(Buzzer, 'lifty_servic>
                self.oled_service = self.create_service(Oled, 'lifty_services/d>

                self.forklift_servo = Servo(12)
                self.buzzer = TonalBuzzer(17)
                self.forklift_servo.value = None
                self.get_logger().info("LIFTY SERVICES STARTED")

                self.fork_state = 1
                self.device = ssd1306(i2c(port=1, address=0x3c), width=128, hei>
                self.device.contrast(1)
                self.oled_startup()

        def display_callback(self, request, response):
                with canvas(self.device, dither = True) as draw:
                        draw.rectangle(self.device.bounding_box, outline='white>
                        message = request.message
                        #text_size = draw.textSize(message)
                        draw.text((30,20), message, fill='white')

                response.response = True
                return response

        def oled_startup(self):
                        with canvas(self.device, dither = True) as draw:
                                #draw.text( (self.device.width - (draw.textsize>
                                draw.rectangle(self.device.bounding_box, outlin>
                                message = 'HELLO LIFT-T'
                                #text_size = draw.textSize(message)
				draw.text((30,20), message, fill='white')

        def set_forklift_callback(self, request, response):
                if request.action == True and self.fork_state == 0:
                        self.forklift_servo.value = 1
                        sleep(0.6)
                        self.forklift_servo.value = None
                        self.fork_state = 1
                        response.response = True
                elif request.action == False and self.fork_state == 1:
                        self.forklift_servo.value = -1
                        sleep(0.5)
                        self.forklift_servo.value = None
                        self.fork_state = 0
                        response.response = False

                return response

        def play_buzzer_callback(self, request, response):
                if request.command == 0 :
                        self.startup()
                elif request.command == 1 :
                        self.alert()
                elif request.command == 2 :
                        self.error()

                response.response = True
                return response

        def startup(self):
                self.buzz('E5',0.05)
                self.buzz('E5',0.05)
                self.buzz('A5',0.1)
                self.buzz('C4',1)

        def alert(self):
		self.buzz('D4',0.05)
                self.buzz('D4',0.05)
                self.buzz('D5',0.1)
                self.buzz('A4',0.1)
                self.buzz('G#4',0.05)
                self.buzz('G4',0.05)
                self.buzz('F4',0.1)
                self.buzz('D4',0.1)
                self.buzz('F4',0.05)
                self.buzz('G4',0.05)


        def error(self):
                self.buzz('E3',3)

        def buzz(self, note, time):
                self.buzzer.play(note)
                sleep(time)
                self.buzzer.stop()
                sleep(time)

def main(args=None):
        rclpy.init(args=args)
        lifty_services = LiftyServices()
        rclpy.spin(lifty_services)
        lifty_services.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()
