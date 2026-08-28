# A significant amount of this file is originally from
# https://github.com/RyanPaulMcKenna/onRobot/blob/9961464d/onRobot/onRobot/gripper.py
# under the MIT License.

from io import BytesIO
from typing import Literal

import pycurl


class VG10:
    def __init__(self, robot_ip: str, vg_id: int) -> None:
        self.robot_ip = robot_ip
        self.vg_id = vg_id

    def vg10_release(self, channelA: Literal[0, 1], channelB: Literal[0, 1]) -> None:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>vg10_release</methodName>
            <params>
                <param>
                <value><int>{self.vg_id}</int></value>
                </param>
                <param>
                <value><boolean>{channelA}</boolean></value>
                </param>
                <param>
                <value><boolean>{channelB}</boolean></value>
                </param>
            </params>
        </methodCall>"""

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        # headers = ["User-Agent: Python-PycURL", "Accept: application/json"]
        data = xml_request.replace("\r\n", "").encode()
        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f"http://{self.robot_ip}:41414")
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode("utf-8"))

        # Close the cURL object
        curl.close()

    def vg10_grip(self, channel: int, vacuum_percent: float) -> None:
        """
        Args:
            channel: 0 (A), 1 (B), 2 (A and B)
            vacuum_percent: softgrip => 30 = 30%, firm grip => 60 = 60%
        """

        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>vg10_grip</methodName>
            <params>
                <param>
                    <value><int>{self.vg_id}</int></value>
                </param>
                <param>
                    <value><int>{channel}</int></value>
                </param>
                <param>
                    <value><double>{vacuum_percent}</double></value>
                </param>
            </params>
        </methodCall>
        """

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        # headers = ["User-Agent: Python-PycURL", "Accept: application/json"]
        data = xml_request.replace("\r\n", "").encode()
        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f"http://{self.robot_ip}:41414")
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode("utf-8"))

        # Close the cURL object
        curl.close()
