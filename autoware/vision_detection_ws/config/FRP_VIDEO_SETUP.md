# FRP video mappings

The browser uses MediaMTX WHEP/WebRTC. Keep the existing rosbridge mapping and
create two additional FRP tunnels for video:

| Purpose | Protocol | Public endpoint | Local endpoint |
|---|---|---|---|
| Autoware/rosbridge | TCP | `frp-any.com:43141` | `127.0.0.1:9090` |
| WHEP signaling | TCP | `frp-all.com:22075` | `127.0.0.1:8889` |
| WebRTC media | UDP | `frp-fee.com:33815` | `127.0.0.1:33815` |

Keep the existing rosbridge proxy unchanged. Merge these two camera proxies
into the existing `frpc.toml` (server address, control port, and authentication
remain the values supplied by the FRP provider):

```toml
[[proxies]]
name = "camera-whep"
type = "tcp"
localIP = "127.0.0.1"
localPort = 8889
remotePort = 22075

[[proxies]]
name = "camera-webrtc"
type = "udp"
localIP = "127.0.0.1"
localPort = 33815
remotePort = 33815
```

The public UDP port must remain `33815`, since MediaMTX advertises port 33815 in
the ICE candidate. If the FRP provider cannot allocate public UDP 33815, use a
different identical local/public media port and update `webrtcLocalUDPAddress`
in `mediamtx.yml` to that port.

The frontend public camera URL is:

```text
http://frp-all.com:22075/cam/
```

Do not use `wss://` for the camera unless TLS is terminated in front of
MediaMTX. The frontend converts `wss://` camera addresses into `https://` WHEP
requests.

After creating the tunnels, start the stream and verify both paths:

```bash
cd /home/tang/autoware/vision_detection_ws
./run_camera_stream.sh

curl -I http://10.245.188.140:8889/cam/
curl -I http://frp-all.com:22075/cam/
```

Opening either URL in a browser should show the same stream. A successful HTTP
page alone only verifies WHEP signaling; confirm that frames arrive to verify
the UDP media mapping as well.
