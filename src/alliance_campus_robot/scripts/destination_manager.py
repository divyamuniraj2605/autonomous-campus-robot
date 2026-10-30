#!/usr/bin/env python3

import os
import yaml

class DestinationManager:

    def __init__(self):

        with open(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "locations.yaml"),
            "r"
        ) as f:

            self.data = yaml.safe_load(f)

    def find_location(self, spoken_text):

        spoken_text = spoken_text.lower()

        locations = self.data["locations"]

        for key, value in locations.items():

            aliases = value["aliases"]

            for alias in aliases:

                if alias in spoken_text:

                    return {
                        "id": key,
                        "name": value["display_name"],
                        "x": value["x"],
                        "y": value["y"]
                    }

        return None


if __name__ == "__main__":

    dm = DestinationManager()

    while True:

        text = input("Say destination: ")

        result = dm.find_location(text)

        if result:

            print("\nDestination Found")
            print(result)

        else:

            print("Location not found")
