import asyncio
import os
from datetime import datetime
from metratec_rfid import RfidReaderException
from example_utils import get_reader

async def print_inventory_and_save(reader):
    """Connect the reader, run a single inventory, and save only EPCs and timestamps, ensuring no duplicates.
    The total number of tags scanned since running the script will be printed."""
    # Create the file path for the tag list and the counter
    folder = "inv_lists"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    file_path = os.path.join(folder, f"tags.txt")
    counter_file_path = os.path.join(folder, "scanned_count.txt")

    try:
        # Ensure the folder exists
        os.makedirs(folder, exist_ok=True)

        # Read existing EPCs from the file into a set to avoid duplicates
        existing_epcs = set()
        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                for line in file:
                    # Extract EPC from each line (assuming the format is "(epc, timestamp)")
                    epc = line.split(',')[0].strip('() ')
                    existing_epcs.add(epc)

        # Read the current total number of scanned tags
        total_scanned = 0
        if os.path.exists(counter_file_path):
            with open(counter_file_path, "r") as counter_file:
                total_scanned = int(counter_file.read())

        # Open the file in append mode to add new data
        with open(file_path, "a") as file:
            # Connect to the reader
            await reader.connect()

            # Run a single inventory
            inventory = await reader.get_inventory()

            # Collect and write the EPC and timestamp for each tag if it's not already in the file
            new_tags = 0
            for tag_data in inventory:
                epc = tag_data.get("epc", "UNKNOWN")
                current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")         

                if epc not in existing_epcs:
                    # Write the new EPC and timestamp to the file
                    file.write(f"({epc}, {current_time})\n")
                    # Add the new EPC to the set
                    existing_epcs.add(epc)

                    # Print the EPC and timestamp
                    print(f"({epc}, {current_time})")
                    new_tags += 1

            # Update the total scanned count
            total_scanned += new_tags
            with open(counter_file_path, "w") as counter_file:
                counter_file.write(str(total_scanned))

        # Print the total number of tags scanned
        print(f"Total number of tags scanned since running the script: {total_scanned}")
        print(f"Inventory saved to {file_path}")

    except Exception as e:
        print(f"An error occurred: {e}")

# Create a new reader object and run the example function.
reader01 = get_reader()
asyncio.run(print_inventory_and_save(reader01))

