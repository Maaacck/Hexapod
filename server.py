cat > ~/server.py <<'PY'
import asyncio, json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

async def offer(request):
    params = await request.json()
    pc = RTCPeerConnection()

    # Try camera; fall back to test pattern if no /dev/video0
    try:
        player = MediaPlayer("/dev/video0")
    except Exception:
        player = MediaPlayer("testsrc=true", format="lavfi")

    if player.video:
        pc.addTrack(player.video)

    @pc.on("datachannel")
    def on_datachannel(channel):
        if channel.label == "control":
            @channel.on("message")
            def on_message(msg):
                print("CTRL:", msg)  # placeholder for robot input handling

    await pc.setRemoteDescription(RTCSessionDescription(**params))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

app = web.Application()
app.router.add_post("/offer", offer)

if __name__ == "__main__":
    web.run_app(app, host="0.0.0.0", port=8080)
PY
