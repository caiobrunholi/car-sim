if data:
    # The current steering angle of the car
    steering_angle = float(data["steering_angle"])
    # The current throttle of the car, how hard to push peddle
    throttle = float(data["throttle"])
    # The current speed of the car
    speed = float(data["speed"])
    # The current image from the center camera of the car
    image = Image.open(BytesIO(base64.b64decode(data["image"])))
    try:
        image = np.asarray(image)  # from PIL image to numpy array
        image = utils.preprocess(image)  # apply the preprocessing
        image = np.array([image])  # the model expects 4D array

        # predict the steering angle for the image
        steering_angle = float(model.predict(image, batch_size=1))
        # lower the throttle as the speed increases
        # if the speed is above the current speed limit, we are on a downhill.
        # make sure we slow down first and then go back to the original max speed.
        global speed_limit
        if speed > speed_limit:
            speed_limit = MIN_SPEED  # slow down
        else:
            speed_limit = MAX_SPEED
        throttle = 1.0 - steering_angle ** 2 - (speed / speed_limit) ** 2
        valorConvertido = translate(steering_angle, -1.5, 1.5, 0, 255)
        if valorConvertido > 255:
            valorConvertido = 255
        elif valorConvertido < 0:
            valorConvertido = 0
        print('{} {} {} {} {}'.format(steering_angle, throttle, speed, int(valorConvertido), arduino.readline()))
        try:
            arduino.write(struct.pack('>B', int(valorConvertido)))
        except:
            pass
        send_control(steering_angle, throttle)
    except Exception as e:
        print(e)

    # save frame
    if args.image_folder != '':
        timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
        image_filename = os.path.join(args.image_folder, timestamp)
        image.save('{}.jpg'.format(image_filename))
else:

    sio.emit('manual', data={}, skip_sid=True)








def autonomo1(steering_angle, throttle, speed):
    global arduino
    global modo
    valorSerial = 0
    valorConvertido = int(translate(steering_angle, -1.5, 1.5, 2, 254))
    if valorConvertido > 254:
        valorConvertido = 254
    elif valorConvertido < 2:
        valorConvertido = 2
    valorSerial = arduino.readline()
    try:
        valorSerial = int(valorSerial)
    except Exception as e:
        pass
    if (valorSerial == 1):
        modo = 0
    print('{} {} {} {} {}'.format(steering_angle, throttle, speed, valorConvertido, valorSerial))
    try:
        arduino.write(struct.pack('>B', valorConvertido))
    except Exception as e:
        print(e)
    send_control(steering_angle, throttle)





def controlado1(steering_angle, throttle):
    global modo
    try:
        arduino = serial.Serial('/dev/cu.usbmodem1411', 115200)
        potenciometro = arduino.readline()
        potInt = int(potenciometro)
        if (potInt == 255):
            modo = 1
            arduino.write(struct.pack('>B', 0))
            return
        else:
            print('controlado: {}'.format(potInt))
            potInt = translate(potInt, 2, 254, 1.5, -1.5)
            send_control(potInt, throttle)
            arduino.write(struct.pack('>B', 0))
            # print(potInt)
        arduino.close()
    except Exception as e:
        send_control(steering_angle, throttle)