from Tool import Tool

class Inventory:
    
    def __init__(self):
        # Initialize an empty dictionary for tools
        self.off_hand = {}
        self.on_hand = {}
    
    # Add object - everything else uses the tool_name string
    def add_tool(self, tool):
        # Use the tool name as the dictionary key
        self.off_hand[tool.tool_name] = tool
    
    def remove_tool(self, tool_name):
        # Remove the tool by its name (key)
        if tool_name in self.off_hand:
            return self.off_hand.pop(tool_name)
        elif tool_name in self.on_hand:
            return self.on_hand.pop(tool_name)
        else:
            print(f"Tool {tool_name} not found in either hand.")
            return None
        
    def get_tool(self, tool_name):
        if tool_name in self.on_hand:
            return self.on_hand[tool_name]
        elif tool_name in self.off_hand:
            return self.off_hand[tool_name]
        else:
            print(f"Tool {tool_name} not found.")
            return None 

    def move_tool(self, tool_name, destination_hand=None, destination_inventory=None):
        if tool_name:
            
            current_hand = 'off_hand' if tool_name in self.off_hand else 'on_hand'
            # current_hand = 'off_hand' if tool_name in self.off_hand.keys() else 'on_hand'

            tool = self.remove_tool(tool_name)

            if tool:
                if ((destination_hand is None) and (destination_inventory is None)):  # Swap between on_hand and off_hand
                    if current_hand == 'on_hand':
                        self.off_hand[tool.tool_name] = tool  # Move to off-hand
                    else:
                        self.set_on_hand(tool)  # Move to on-hand
                elif destination_inventory:  # Moving between inventories
                    if destination_hand == 'on_hand':
                        destination_inventory.set_on_hand(tool)
                    else:
                        destination_inventory.add_tool(tool)
                else:  # Moving within the same inventory
                    if destination_hand == 'on_hand':
                        self.set_on_hand(tool)
                    elif destination_hand == 'off_hand':
                        self.off_hand[tool_name] = tool

    
    def set_on_hand(self, tool):
        if self.on_hand:
            print(f"Cannot place {tool.tool_name} in on-hand. {list(self.on_hand.keys())[0]} is already on-hand.")
        else:
            self.on_hand[tool.tool_name] = tool
    
    def __repr__(self):
        on_hand_tools = list(self.on_hand.keys()) or ["None"]
        off_hand_tools = list(self.off_hand.keys()) or ["None"]
        return (f"Inventory:\n"
                f"On-hand: {on_hand_tools}\n"
                f"Off-hand: {off_hand_tools}")
