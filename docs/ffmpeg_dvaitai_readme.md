# 🎬 FFmpeg Quick Reference — DvaitAI Video Workflow
> 📘 Complete guide for merging, fixing, syncing, and enhancing videos with audio using FFmpeg.

---

## ⚙️ 1️⃣ Basic Info

| Task | Command |
|------|----------|
| Check video info | `ffmpeg -i video.mp4` |
| Convert WebM → MP4 | `ffmpeg -i input.webm -c:v libx264 -c:a aac -strict experimental output.mp4` |
| Extract audio only | `ffmpeg -i input.mp4 -q:a 0 -map a output.mp3` |
| Replace audio | `ffmpeg -i video.mp4 -i audio.mp3 -c:v copy -map 0:v:0 -map 1:a:0 -shortest output.mp4` |

---

## 🎥 2️⃣ Merge Two Videos (One After Another)

### 🧩 If both are same format (MP4 + MP4)
```bash
echo "file 'intro.mp4'" > list.txt
echo "file 'main_video.mp4'" >> list.txt
ffmpeg -f concat -safe 0 -i list.txt -c copy merged.mp4
```

---

### 🧩 If formats differ (e.g., .webm + .mp4)
#### Step 1 – Convert both to same format:
```bash
ffmpeg -i intro.mp4 -c:v libx264 -c:a aac intro_fixed.mp4
ffmpeg -i main_video.webm -c:v libx264 -c:a aac main_fixed.mp4
```

#### Step 2 – Merge:
```bash
echo "file 'intro_fixed.mp4'" > list.txt
echo "file 'main_fixed.mp4'" >> list.txt
ffmpeg -f concat -safe 0 -i list.txt -c copy merged_dvaitai_video.mp4
```

✅ Keeps both audios, perfect sync.

---

## 🎧 3️⃣ Add Voiceover to Video
```bash
ffmpeg -i video.mp4 -i voice.mp3 -c:v copy -map 0:v:0 -map 1:a:0 -shortest final_voiced_video.mp4
```

💡 If you want to *replace* the video’s audio entirely, this command is perfect.

---

## 🎵 4️⃣ Add Background Music Under Voice
```bash
ffmpeg -i voice.mp3 -i music.mp3 -filter_complex "[0:a][1:a]amix=inputs=2:duration=first:dropout_transition=2" -c:a aac voice_with_music.mp3
```
🎚️ Adjust music volume:
```bash
ffmpeg -i voice.mp3 -i music.mp3 -filter_complex "[1:a]volume=0.3[a1];[0:a][a1]amix=inputs=2:duration=first" -c:a aac voice_with_music.mp3
```

---

## ⏱️ 5️⃣ Sync Audio with Video Length

### 🐢 Stretch Audio to Match Longer Video
```bash
ffmpeg -i voice.mp3 -filter:a "atempo=0.8" stretched_voice.mp3
```

### 🧍 Add Silence to Match Video
```bash
ffmpeg -i voice.mp3 -af "apad=pad_dur=30" padded_voice.mp3
```

---

## 🌈 6️⃣ Add Smooth Fade Transition Between Two Videos
```bash
ffmpeg -i intro.mp4 -i main.mp4 -filter_complex "[0:v][0:a][1:v][1:a]xfade=transition=fade:duration=1:offset=5,acrossfade=d=1[v][a]" -map "[v]" -map "[a]" -c:v libx264 -c:a aac merged_fade.mp4
```
🕐 `offset=5` → transition starts at 5 seconds  
🎧 `acrossfade=d=1` → smooth audio crossfade

---

## 🧠 7️⃣ Fix Frame Duplication / Lag
If FFmpeg says “More than 10000 frames duplicated”:
```bash
ffmpeg -i video.mp4 -r 30 -vf "scale=1280:720" -c:v libx264 -c:a aac fixed_video.mp4
```
Then re-merge normally.

---

## 🔉 8️⃣ Combine Two Audio Files Sequentially
```bash
echo "file 'intro.mp3'" > audio_list.txt
echo "file 'main.mp3'" >> audio_list.txt
ffmpeg -f concat -safe 0 -i audio_list.txt -c copy full_audio.mp3
```

---

## 🪄 9️⃣ Add Fade-in/Fade-out to Audio
```bash
ffmpeg -i audio.mp3 -af "afade=t=in:ss=0:d=2,afade=t=out:st=90:d=3" faded_audio.mp3
```
🎵 Fade in for 2s, fade out starting at 90s for 3s.

---

## 🧰 10️⃣ Convert or Compress Video for YouTube / Social Media
```bash
ffmpeg -i input.mp4 -vcodec libx264 -crf 23 -preset medium -acodec aac -b:a 192k -movflags +faststart output_compressed.mp4
```

---

## 🧾 11️⃣ Extract or Replace Specific Sections
Trim part of a video:
```bash
ffmpeg -ss 00:00:10 -to 00:00:20 -i video.mp4 -c copy clip.mp4
```
Cut and replace part of a video with another:
```bash
ffmpeg -i part1.mp4 -i part2.mp4 -filter_complex "[0:v][0:a][1:v][1:a]concat=n=2:v=1:a=1[v][a]" -map "[v]" -map "[a]" final.mp4
```

---

## 🧠 12️⃣ Bonus: Text Overlay (Optional)
Add text like “DvaitAI - Day 6” at top:
```bash
ffmpeg -i input.mp4 -vf "drawtext=text='DvaitAI - Day 6':fontcolor=white:fontsize=36:x=(w-text_w)/2:y=20" -codec:a copy output.mp4
```

---

## 🏁 Example Full Workflow (your most common case)
You have:
```
intro.mp4
dvaitai_video.webm
dvaitai_voice_final.mp3
```

You want:
```
final_dvaitai_video.mp4
```

Run this sequence:
```bash
# Step 1: Convert webm to mp4
ffmpeg -i dvaitai_video.webm -c:v libx264 -c:a aac main_fixed.mp4

# Step 2: Merge intro and main video
echo "file 'intro.mp4'" > list.txt
echo "file 'main_fixed.mp4'" >> list.txt
ffmpeg -f concat -safe 0 -i list.txt -c copy merged.mp4

# Step 3: Add voice
ffmpeg -i merged.mp4 -i dvaitai_voice_final.mp3 -c:v copy -map 0:v:0 -map 1:a:0 -shortest final_dvaitai_video.mp4
```

---

### ✅ Output:
🎬 `final_dvaitai_video.mp4`  
- Clean merge  
- Perfect voice sync  
- Audio preserved  
- Smooth playback (no frame dupes)
