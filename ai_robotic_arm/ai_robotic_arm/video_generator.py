# import asyncio
# import edge_tts
# from pydub import AudioSegment

# # === Step 1: Generate Male AI Voice ===
# text = """
# Over the last few days, we’ve built the robotic arm, connected it with ROS 2, and added AI control logic.

# But today, something special happened —
# DvaitAI finally got a face.

# I designed a Tkinter-based dashboard that lets me control the robotic arm with a simple, intuitive interface.

# It connects directly with ROS 2 topics,
# so when I click a button like Move Up, Pick, or Drop, the command is instantly published to the robot,
# and the status updates appear here on the screen in real time.

# This small interface represents a huge shift —
# from lines of code to visible intelligence.
# Now I can literally see what my system thinks, and feel how it responds.

# This dashboard is the bridge between mind and machine,
# between logic and experience.

# In the next video, I’ll enable AI Mode,
# where the arm will start making its own decisions based on patterns and data —
# moving closer to autonomy.

# Every day, DvaitAI is learning, adapting, and becoming more intelligent.

# 🧠 Motivation of the Day:
# “When intelligence becomes visible, it becomes usable.
# But when intelligence becomes emotional, it becomes unstoppable.”

# Thanks for watching —
# this is DvaitAI,
# where intelligence meets duality.
# """

# VOICE = "en-IN-PrabhatNeural"
# VOICE_FILE = "dvaitai_voice_male.mp3"
# FINAL_AUDIO = "dvaitai_voice_final.mp3"
# VIDEO_LENGTH = 112 * 1000  # milliseconds

# async def gen_voice():
#     communicate = edge_tts.Communicate(text, VOICE)
#     await communicate.save(VOICE_FILE)

# asyncio.run(gen_voice())

# # === Step 2: Add Intro/Outro Music and Match Length ===
# # You can replace these with your own music files (MP3 or WAV)
# intro_music = AudioSegment.silent(duration=3000)  # placeholder 3 sec silence
# outro_music = AudioSegment.silent(duration=4000)  # placeholder 4 sec silence

# # Load generated voice
# voice = AudioSegment.from_file(VOICE_FILE)

# # Add intro/outro (replace silence with actual music later)
# final_audio = intro_music + voice + outro_music

# # Pad or trim to exactly 1:52 (112 sec)
# if len(final_audio) < VIDEO_LENGTH:
#     silence = AudioSegment.silent(duration=VIDEO_LENGTH - len(final_audio))
#     final_audio += silence
# else:
#     final_audio = final_audio[:VIDEO_LENGTH]

# # Export
# final_audio.export(FINAL_AUDIO, format="mp3")
# print("✅ Final voice with intro/outro saved as:", FINAL_AUDIO)

from moviepy import VideoFileClip, AudioFileClip

# Load video and final voice
video = VideoFileClip("/home/nk/Videos/Screencasts/Screencast from 07-11-25 08:15:39 PM IST.webm")
audio = AudioFileClip("dvaitai_voice_final.mp3")

# Sync lengths
if audio.duration < video.duration:
    # Add silence if audio is slightly shorter
    from pydub import AudioSegment
    voice = AudioSegment.from_file("dvaitai_voice_final.mp3")
    silence = AudioSegment.silent(duration=(video.duration - audio.duration) * 1000)
    padded = voice + silence
    padded.export("dvaitai_voice_padded.mp3", format="mp3")
    audio = AudioFileClip("dvaitai_voice_padded.mp3")

# Merge
final = video.with_audio(audio)

# Export final video
final.write_videofile(
    "final_dvaitai_video.webm",
    codec="libvpx-vp9",
    audio_codec="libvorbis",
    threads=4,
    fps=video.fps
)
