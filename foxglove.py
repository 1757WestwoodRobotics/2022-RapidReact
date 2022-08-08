import asyncio, json, time
from enum import Enum, auto
from typing import Tuple

import wpilib


from foxglove_websocket import run_cancellable
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener
from foxglove_websocket.types import ChannelId

from util.convenientmath import get_quaternion_from_euler


class FoxglovePublisher:
    class FoxgloveType(Enum):
        Bool = auto()
        Number = auto()
        Pose2d = auto()

    def __init__(self, **topic_sub: Tuple[str, FoxgloveType]):
        print(topic_sub)
        self.topics = topic_sub

    async def foxglove_serv(self):
        print("initialization of foxglove...")

        class Listener(FoxgloveServerListener):
            def on_subscribe(self, server: FoxgloveServer, channel_id: ChannelId):
                print("First client subscribed to", channel_id)

            def on_unsubscribe(self, server: FoxgloveServer, channel_id: ChannelId):
                print("Last client unsubscribed from", channel_id)

        async with FoxgloveServer("0.0.0.0", 5804, "1757 Robot") as server:
            server.set_listener(Listener())

            table = wpilib.SmartDashboard

            self.topic_map = {}

            for name in self.topics:
                val = self.topics[name]
                type = val[1]

                if type == FoxglovePublisher.FoxgloveType.Bool:
                    self.topic_map[name] = await server.add_channel(
                        {
                            "topic": f"/{name}",
                            "encoding": "json",
                            "schemaName": "Bool",
                            "schema": json.dumps(
                                {
                                    "type": "object",
                                    "properties": {"val": {"type": "boolean"}},
                                }
                            ),
                        }
                    )
                elif type == FoxglovePublisher.FoxgloveType.Number:
                    self.topic_map[name] = await server.add_channel(
                        {
                            "topic": f"/{name}",
                            "encoding": "json",
                            "schemaName": "Number",
                            "schema": json.dumps(
                                {
                                    "type": "object",
                                    "properties": {"val": {"type": "number"}},
                                }
                            ),
                        }
                    )
                elif type == FoxglovePublisher.FoxgloveType.Pose2d:
                    self.topic_map[name] = await server.add_channel(
                        {
                            "topic": f"/{name}",
                            "encoding": "json",
                            "schemaName": "foxglove.PoseInFrame",
                        }
                    )

            while True:
                await asyncio.sleep(0.05)
                for name in self.topics:
                    val = self.topics[name][0]
                    type = self.topics[name][1]

                    if type == FoxglovePublisher.FoxgloveType.Bool:
                        await server.send_message(
                            self.topic_map[name],
                            time.time_ns(),
                            json.dumps({"val": table.getBoolean(val, False)}).encode(
                                "utf8"
                            ),
                        )
                    elif type == FoxglovePublisher.FoxgloveType.Number:
                        await server.send_message(
                            self.topic_map[name],
                            time.time_ns(),
                            json.dumps({"val": table.getNumber(val, 0)}).encode("utf8"),
                        )
                    elif type == FoxglovePublisher.FoxgloveType.Pose2d:
                        [x, y, theta] = table.getNumberArray(val, [0, 0, 0])
                        rot = get_quaternion_from_euler(0, 0, theta)
                        await server.send_message(
                            self.topic_map[name],
                            time.time_ns(),
                            json.dumps(
                                {
                                    "timestamp": {
                                        "sec": int(time.time()),
                                        "nsec": time.time_ns() % 1e9,
                                    },
                                    "frame_id": "field",
                                    "pose": {
                                        "position": {"x": x, "y": y, "z": 0},
                                        "orientation": {
                                            "x": rot[0],
                                            "y": rot[1],
                                            "z": rot[2],
                                            "w": rot[3],
                                        },
                                    },
                                }
                            ).encode("utf8"),
                        )

    def run_bot(self, bot):
        async def bot_cmd():
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, wpilib.run, bot)

        async def run_all():
            await asyncio.wait([bot_cmd(), self.foxglove_serv()])

        run_cancellable(run_all())
