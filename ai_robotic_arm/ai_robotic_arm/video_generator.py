import asyncio
import edge_tts
from pydub import AudioSegment

# === Step 1: Generate Male AI Voice ===
text = """
Welcome to DvaitAI — where intelligence will grow with discipline.

This is a robotic arm simulator powered by Python and ROS 2,
with a desktop dashboard built in Tkinter and a live telemetry chart.

First, Manual Mode.
When I press a button, a command is published on a ROS 2 topic,
and the arm responds in real time.
Every action is also logged to a CSV file for analysis.

Now, Autopilot Mode.
Here the system generates simple pattern or random actions on its own,
so you can see continuous motion without manual input.
There’s no trained AI model yet — this is a baseline autopilot for demo and testing.

Down here is the live chart.
It plots the last set of joint angles in real time,
so you can watch motion turn into data.
I can also pause the graph when I want to inspect behavior closely.

What can you take from this video?
You can build a clean ROS 2 pipeline end to end:
publish commands, receive feedback, log telemetry,
and visualize it in a practical dashboard.
This is a solid foundation to add real intelligence later.

With this version, DvaitAI reaches a stable v1:
ROS 2 control, manual dashboard, autopilot patterns,
CSV logging, and live visualization.

We’ll pause this project here and start planning v2:
adding real decision-making — for example,
rule-based strategies first, then model-based control,
and eventually learning approaches when data is ready.


Thanks for watching —
this is DvaitAI,
where intelligence meets duality.
"""

VOICE = "en-IN-PrabhatNeural"
VOICE_FILE = "dvaitai_voice_male.mp3"
FINAL_AUDIO = "dvaitai_voice_final.mp3"
VIDEO_LENGTH = 112 * 1000  # milliseconds

async def gen_voice():
    communicate = edge_tts.Communicate(text, VOICE)
    await communicate.save(VOICE_FILE)

asyncio.run(gen_voice())

# === Step 2: Add Intro/Outro Music and Match Length ===
# You can replace these with your own music files (MP3 or WAV)
intro_music = AudioSegment.silent(duration=3000)  # placeholder 3 sec silence
outro_music = AudioSegment.silent(duration=4000)  # placeholder 4 sec silence

# Load generated voice
voice = AudioSegment.from_file(VOICE_FILE)

# Add intro/outro (replace silence with actual music later)
final_audio = intro_music + voice + outro_music

# Pad or trim to exactly 1:52 (112 sec)
if len(final_audio) < VIDEO_LENGTH:
    silence = AudioSegment.silent(duration=VIDEO_LENGTH - len(final_audio))
    final_audio += silence
else:
    final_audio = final_audio[:VIDEO_LENGTH]

# Export
final_audio.export(FINAL_AUDIO, format="mp3")
print("✅ Final voice with intro/outro saved as:", FINAL_AUDIO)

# from moviepy import VideoFileClip, AudioFileClip

# # Load video and final voice
# video = VideoFileClip("/home/nk/Videos/Screencasts/Screencast from 07-11-25 08:15:39 PM IST.webm")
# audio = AudioFileClip("dvaitai_voice_final.mp3")

# # Sync lengths
# if audio.duration < video.duration:
#     # Add silence if audio is slightly shorter
#     from pydub import AudioSegment
#     voice = AudioSegment.from_file("dvaitai_voice_final.mp3")
#     silence = AudioSegment.silent(duration=(video.duration - audio.duration) * 1000)
#     padded = voice + silence
#     padded.export("dvaitai_voice_padded.mp3", format="mp3")
#     audio = AudioFileClip("dvaitai_voice_padded.mp3")

# # Merge
# final = video.with_audio(audio)

# # Export final video
# final.write_videofile(
#     "final_dvaitai_video.webm",
#     codec="libvpx-vp9",
#     audio_codec="libvorbis",
#     threads=4,
#     fps=video.fps
# )
