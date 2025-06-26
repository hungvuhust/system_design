#!/bin/bash

# Kiểm tra các dependencies
check_dependencies() {
    local missing_deps=()
    
    # Kiểm tra pandoc
    if ! command -v pandoc &> /dev/null; then
        missing_deps+=("pandoc")
    fi
    
    # Kiểm tra texlive
    if ! command -v pdflatex &> /dev/null; then
        missing_deps+=("texlive-full")
    fi

    # Kiểm tra fonts
    if ! fc-list | grep -i "Liberation Sans" > /dev/null; then
        missing_deps+=("fonts-liberation")
    fi
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        echo "Các dependencies sau cần được cài đặt:"
        printf '%s\n' "${missing_deps[@]}"
        echo "Bạn có thể cài đặt bằng lệnh:"
        echo "sudo apt-get install ${missing_deps[*]}"
        exit 1
    fi
}

# Kiểm tra dependencies
check_dependencies

# Tạo thư mục output nếu chưa tồn tại
mkdir -p output

# Tạo file tạm thời để chứa nội dung tổng hợp
TMP_FILE="output/tmp_combined.md"
FINAL_PDF="output/design_patterns_ros2.pdf"

# Xóa file tạm nếu đã tồn tại
rm -f $TMP_FILE

# Tạo file header.tex cho font và mục lục configuration
cat << 'EOF' > output/header.tex
\usepackage{fontspec}
\setmainfont{NotoSans}[
    Path = /usr/share/fonts/truetype/noto/,
    Extension = .ttf,
    UprightFont = *-Regular,
    BoldFont = *-Bold,
    ItalicFont = *-Italic,
    BoldItalicFont = *-BoldItalic
]
\setmonofont{NotoSansMono}[
    Path = /usr/share/fonts/truetype/noto/,
    Extension = .ttf,
    UprightFont = *-Regular,
    BoldFont = *-Bold
]

% Cấu hình cho mục lục
\usepackage{tocloft}
\renewcommand{\cftsecfont}{\normalfont}
\renewcommand{\cftsecpagefont}{\normalfont}
\setlength{\cftbeforesecskip}{2pt}
\setlength{\cftbeforesubsecskip}{2pt}

% Chỉ đánh số cho chapter (level 0)
\setcounter{secnumdepth}{0}
\setcounter{tocdepth}{3}

% Định dạng lại numbering cho section
\makeatletter
\renewcommand{\thesection}{\arabic{section}}
\renewcommand{\thesubsection}{}
\renewcommand{\thesubsubsection}{}
\makeatother

% Cấu hình cho code blocks
\usepackage{fancyvrb}
\fvset{fontsize=\small}
\DefineVerbatimEnvironment{Highlighting}{Verbatim}{commandchars=\\\{\},fontsize=\small}

% Cấu hình cho hyperlinks
\usepackage{hyperref}
\hypersetup{
    colorlinks=true,
    linkcolor=blue,
    filecolor=magenta,
    urlcolor=blue,
    pdftitle={Design Patterns trong ROS2 và Robotics},
    pdfauthor={Hưng Vũ},
    pdfsubject={Design Patterns},
    pdfkeywords={ROS2, Design Patterns, Robotics}
}
EOF

# Thêm trang bìa và thông tin
cat << 'EOF' > $TMP_FILE
---
title: "DESIGN PATTERNS TRONG ROBOTICS"
author: "Hưng Vũ"
date: \today
documentclass: report
classoption: oneside
geometry: margin=1in
fontsize: 11pt
---

\tableofcontents
\newpage

EOF

# Thêm toàn bộ nội dung README
cat README.md >> $TMP_FILE

# Thêm phân cách section
echo -e "\n\n# Phần 1: Creational Patterns\n" >> $TMP_FILE

# Thêm các Creational Patterns
for pattern in abstract_factory builder factory prototype singleton; do
    echo -e "\n## $(echo $pattern | tr '_' ' ' | sed 's/\b\(.\)/\u\1/g') Pattern\n" >> $TMP_FILE
    cat "patterns/creational/$pattern.md" | tail -n +2 >> $TMP_FILE
done

# Thêm phân cách section
echo -e "\n\n# Phần 2: Structural Patterns\n" >> $TMP_FILE

# Thêm các Structural Patterns
for pattern in adapter bridge composite decorator facade flyweight proxy; do
    echo -e "\n## $(echo $pattern | tr '_' ' ' | sed 's/\b\(.\)/\u\1/g') Pattern\n" >> $TMP_FILE
    cat "patterns/structural/$pattern.md" | tail -n +2 >> $TMP_FILE
done

# Thêm phân cách section
echo -e "\n\n# Phần 3: Behavioral Patterns\n" >> $TMP_FILE

# Thêm các Behavioral Patterns
for pattern in chain_of_responsibility command interpreter iterator mediator memento observer state strategy template_method visitor; do
    echo -e "\n## $(echo $pattern | tr '_' ' ' | sed 's/\b\(.\)/\u\1/g') Pattern\n" >> $TMP_FILE
    cat "patterns/behavioral/$pattern.md" | tail -n +2 >> $TMP_FILE
done

# Tạo PDF với pandoc
echo "Đang tạo PDF..."
pandoc $TMP_FILE \
    -f markdown+raw_tex \
    -t pdf \
    --toc \
    --toc-depth=3 \
    --top-level-division=chapter \
    --number-sections \
    --highlight-style=tango \
    --pdf-engine=xelatex \
    -V mainfont="NotoSans-Regular" \
    -V monofont="NotoSansMono-Regular" \
    -V lang=vi \
    -V babel-lang=vietnamese \
    -H output/header.tex \
    -o $FINAL_PDF

# Xóa files tạm
rm $TMP_FILE
rm output/header.tex

echo "Đã tạo file PDF tại: $FINAL_PDF"