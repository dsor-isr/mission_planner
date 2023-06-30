import time
import argparse
import roslibpy


print("Reading arguments")
try:
    # Create the parser
    parser = argparse.ArgumentParser()
    # Add an argument
    parser.add_argument('--vehicle', type=str, required=True)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--port', type=int)
    # Parse the argument
    args = parser.parse_args()
except:
    print("Couldn't read arguments, try: python3 watchdog_client_latency.py --vehicle [VEHICLE_IP]")
    print("(optional) --duration [DURATION] --port [PORT]")

print("Creating ROS client")
port = args.port if args.port else 10111
try:
    client = roslibpy.Ros(host=args.vehicle, port=port)
    print(client.is_connected)
except:
    print("Couldn't create ROS client")

def receive_message(msg):
    age = int(time.time() * 1000) - msg['data']
    try:
        #log.info('Age of message: %6dms', age)
        print("[" + str(time.time()) + "]\tAge of message: " + str(age) + "ms")
    except:
        print("Couldn't print received msg:")
        print(msg)

print("Starting Publisher")
publisher = roslibpy.Topic(client, '/' + args.vehicle + '0/watchdog/client', 'std_msgs/UInt64')
publisher.advertise()

print("Starting Subscriber")
subscriber = roslibpy.Topic(client, '/' + args.vehicle + '0/watchdog/client', 'std_msgs/UInt64')
subscriber.subscribe(receive_message)

def publish_message():
    publisher.publish(dict(data=int(time.time() * 1000)))
    client.call_later(args.duration if args.duration else 0.5, publish_message)

print("Running forever, do CTRL+c to stop...\n")
print("TIME\t\t\tTIME_BETWEEN_MESSAGES")
client.on_ready(publish_message)
client.run_forever()

print("End of Safeties")
