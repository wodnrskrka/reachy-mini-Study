import pydub
import os
import numpy as np

# 1. 경로 설정 (반드시 파일 확장자 .exe까지 포함하세요)
ffmpeg_path = r"C:\python\reachy-mini-Study\ffmpeg.exe"
ffprobe_path = r"C:\python\reachy-mini-Study\ffprobe.exe"

pydub.AudioSegment.converter = ffmpeg_path
pydub.AudioSegment.ffprobe = ffprobe_path

# 2. 시스템 환경 변수(PATH)에 현재 폴더 강제 추가 (이게 핵심입니다)
os.environ["PATH"] += os.pathsep + r"C:\python\reachy-mini-Study"

def speak(text, mini):
    print(f"[로봇 답변]: {text}")
    tts = gTTS(text=text, lang='ko')
    
    # 임시 파일로 저장 (메모리에서 바로 변환 시 ffmpeg 에러가 더 잦음)
    temp_file = "temp_speech.mp3"
    tts.save(temp_file)
    
    try:
        # 파일로부터 읽기
        audio = pydub.AudioSegment.from_file(temp_file, format="mp3")
        audio_array = np.array(audio.get_array_of_samples())

        chunk_size = 1024
        for i in range(0, len(audio_array), chunk_size):
            chunk = audio_array[i:i+chunk_size]
            mini.media.push_audio_sample(chunk)
    finally:
        # 작업 후 임시 파일 삭제 시도 (선택 사항)
        if os.path.exists(temp_file):
            pass