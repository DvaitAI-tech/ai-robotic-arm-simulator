# 🧠 Git Multi-Account Setup & Migration Guide (DvaitAI-Tech)

This document summarizes all commands used to migrate from the **work account (kukrumku)** to the **DvaitAI-tech** account and set up multiple GitHub SSH identities on one system.

---

## ⚙️ SSH Configuration for Multiple GitHub Accounts

Edit your SSH config file:
```bash
nano ~/.ssh/config
```

Add these entries:
```bash
# Personal GitHub account
Host github.com-personal
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_personal

# Work GitHub account
Host github.com-work
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_work

# DvaitAI GitHub account
Host github.com-dvaitai
    HostName github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_dvaitai
```

---

## 🔑 Add SSH Key to GitHub (DvaitAI)
Copy and add this key to your **DvaitAI-tech** GitHub account:
```bash
cat ~/.ssh/id_ed25519_dvaitai.pub
```

---

## 🚀 Migrate Repository (Work ➜ DvaitAI)

Clone the old repo as a mirror:
```bash
git clone --mirror git@github.com-work:kukrumku/ai-robotic-arm-simulator.git
cd ai-robotic-arm-simulator.git
```

Push everything (branches, tags, commits) to new repo:
```bash
git push --mirror git@github.com-dvaitai:DvaitAI-tech/ai-robotic-arm-simulator.git
```

(Optional) clean up:
```bash
cd ..
rm -rf ai-robotic-arm-simulator.git
```

---

## 🔁 Reconfigure Local Repository to Use DvaitAI

Go to your repo:
```bash
cd ~/ros2_ws/src/ai-robotic-arm-simulator
```

Update remote URL:
```bash
git remote set-url origin git@github.com-dvaitai:DvaitAI-tech/ai-robotic-arm-simulator.git
```

Verify:
```bash
git remote -v
```

---

## 🧩 Set Local Git Identity to DvaitAI
```bash
git config user.name "DvaitAI"
git config user.email "your_mail@gmail.com"
```

Check:
```bash
git config user.name
git config user.email
```

---

## 🔍 Test SSH and Push Access
```bash
ssh -T git@github.com-dvaitai
```

Expected:
```
Hi DvaitAI-tech! You've successfully authenticated, but GitHub does not provide shell access.
```

---

## 🧪 Verify Everything
```bash
echo "DvaitAI test connection" >> test.txt
git add test.txt
git commit -m "Test commit from DvaitAI setup"
git push origin main
```

Then confirm the commit shows as **DvaitAI** on GitHub.

---

### ✅ Final Setup Summary

| Setting | Value |
|----------|--------|
| Remote | `git@github.com-dvaitai:DvaitAI-tech/ai-robotic-arm-simulator.git` |
| SSH Key | `~/.ssh/id_ed25519_dvaitai` |
| Git User | `DvaitAI` |
| Git Email | `your_mail@gmail.com` |
| Auth Verified | ✅ via `ssh -T git@github.com-dvaitai` |

---

### 🧰 Notes
- Always verify your identity per repo:  
  `git config user.name && git config user.email`
- Use `git@github.com-dvaitai:` for DvaitAI projects.
- Use `git@github.com-work:` for work projects.
- Use `git@github.com-personal:` for personal projects.

---

**Author:** `DvaitAI-tech`  
**Last Updated:** November 2025
