class Tool:
    def __init__(self, tool_id: int, tool_name: str, tool_position: float)->None:
        print("created tool")
        self.tool_id = tool_id
        self.tool_name = tool_name
        self.tool_position = tool_position

    def get_position(self):
        return self.tool_position

    def __repr__(self):
        return f"Tool(class={self.tool_id}, name='{self.tool_name}', position={self.tool_position})"

