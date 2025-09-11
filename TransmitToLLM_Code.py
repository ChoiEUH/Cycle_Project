import serial
from openai import OpenAI
import pyttsx3
import time

client = OpenAI(api_key="api키를 입력")
tts_engine = pyttsx3.init()
tts_engine.setProperty("rate", 160)
tts_engine.setProperty("volume", 1.0)

ser = serial.Serial(port="COM3", baudrate=115200, timeout=10)

latest_roll = None
latest_pitch = None
latest_distance = None

tts_busy = False

print("🚲 자전거 안전 모니터링 시작 (STM32 시리얼 연결됨)")

def get_model_response(roll, pitch, distance):
    prompt = f"""
    자전거 센서 데이터:
    - 롤(좌우 기울기): {roll:.2f}°
    - 피치(앞뒤 기울기): {pitch:.2f}°
    - 뒤 장애물 거리: {distance} cm

    간단히 안내 메시지를 주세요.
    """
    try:
        resp = client.chat.completions.create(
            model="gpt-4o-mini",
            messages=[
                {"role": "system", "content": "너는 자전거 안전 보조 AI 어시스턴트다."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=50
        )
        return resp.choices[0].message.content.strip()
    except Exception as e:
        print("⚠️ AI 호출 실패:", e)
        return None

def speak(msg):
    global tts_busy
    tts_busy = True
    tts_engine.say(msg)
    tts_engine.runAndWait()
    tts_busy = False

while True:
    try:
        line = ser.readline().decode(errors="ignore").replace("\x00", "").strip()
        if not line:
            continue

        parts = line.split(",")

        if len(parts) == 2:  # roll,pitch
            latest_roll = float(parts[0])
            latest_pitch = float(parts[1])
        elif len(parts) == 1:  # distance
            try:
                latest_distance = int(parts[0])
            except ValueError:
                continue
        else:
            continue

     
        if latest_roll is not None and latest_pitch is not None and latest_distance is not None and not tts_busy:
            print(f"[수신] roll={latest_roll}, pitch={latest_pitch}, distance={latest_distance}")
            alert_msg = get_model_response(latest_roll, latest_pitch, latest_distance)
            if alert_msg:
                print(f"[AI 알림] {alert_msg}")
                speak(alert_msg)

           
            latest_roll = None
            latest_pitch = None
            latest_distance = None

          
            time.sleep(0.2)

    except Exception as e:
        print("데이터 파싱 오류:", repr(line), e)
