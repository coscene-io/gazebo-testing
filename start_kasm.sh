#!/bin/bash

# 设置VNC密码
export VNC_PW=123456

# 创建xstartup文件
mkdir -p ~/.vnc
cat > ~/.vnc/xstartup << EOF
#!/bin/sh
xrdb $HOME/.Xresources
startxfce4 &
EOF
chmod +x ~/.vnc/xstartup

# 启动KasmVNC服务器
kasmvncserver -select-de xfce -geometry 1280x720 -depth 24 -passwd $VNC_PW -websocket $DISPLAY

# 保持前台运行
tail -f /dev/null 