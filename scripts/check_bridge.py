#!/usr/bin/env python3
"""Test the local Foxglove protocol directly; no account or cloud upload involved."""
import base64
import json
import os
import socket
import struct
import time

s = socket.create_connection(('127.0.0.1',8766),timeout=10)
key = base64.b64encode(os.urandom(16)).decode()
s.sendall((f'GET / HTTP/1.1\r\nHost: localhost:8766\r\nUpgrade: websocket\r\n'
           f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
           'Sec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1\r\n\r\n').encode())
buffer=b''
while b'\r\n\r\n' not in buffer: buffer+=s.recv(4096)
header,buffer=buffer.split(b'\r\n\r\n',1)
assert b'101 Switching Protocols' in header,(header+buffer).decode(errors='replace')

def read(n):
    global buffer
    while len(buffer)<n:
        chunk=s.recv(65536)
        if not chunk:raise RuntimeError('Bridge closed connection')
        buffer+=chunk
    result,buffer=buffer[:n],buffer[n:]
    return result

def receive():
    a,b=read(2);length=b&127
    if length==126:length=struct.unpack('!H',read(2))[0]
    if length==127:length=struct.unpack('!Q',read(8))[0]
    if b&128:raise RuntimeError('Unexpected masked server frame')
    return a&15,read(length)

def send_json(value):
    payload=json.dumps(value).encode();mask=os.urandom(4)
    prefix=bytes([0x81,0x80|len(payload)]) if len(payload)<126 else bytes([0x81,0xfe])+struct.pack('!H',len(payload))
    s.sendall(prefix+mask+bytes(v^mask[i%4] for i,v in enumerate(payload)))

topics={};server=None;received=False;subscribed=False
deadline=time.monotonic()+20
while time.monotonic()<deadline:
    opcode,payload=receive()
    if opcode==1:
        data=json.loads(payload)
        if data.get('op')=='serverInfo':server=data.get('name')
        if data.get('op')=='advertise':
            topics.update({c['topic']:c['id'] for c in data['channels']})
    if '/lidar' in topics and not subscribed:
        send_json({'op':'subscribe','subscriptions':[{'id':1,'channelId':topics['/lidar']}]})
        subscribed=True
    if opcode==2 and payload and payload[0]==1:received=True
    if received and all(t in topics for t in ['/lidar','/costmap','/map','/path','/cmd_vel','/odom/filtered']):break
result={'websocket_upgrade':True,'server':server,'topics':sorted(topics),'lidar_binary_message_received':received}
print(json.dumps(result,indent=2))
s.close()
raise SystemExit(0 if received and all(t in topics for t in ['/lidar','/costmap','/map','/path','/cmd_vel','/odom/filtered']) else 1)
