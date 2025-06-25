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

# Tạo file header.tex tạm thời
cat > header.tex << 'EOL'
\usepackage{fancyvrb}
\fvset{fontsize=\footnotesize}
\DefineVerbatimEnvironment{Highlighting}{Verbatim}{commandchars=\\\{\},fontsize=\footnotesize}
\RecustomVerbatimEnvironment{verbatim}{Verbatim}{fontsize=\footnotesize}
EOL

# Chuyển đổi từng file markdown sang PDF
convert_markdown_to_pdf() {
    local input_file=$1
    local output_file=$2
    
    echo "Đang chuyển đổi $input_file sang PDF..."
    
    # Chuyển markdown sang PDF trực tiếp bằng pandoc
    pandoc "$input_file" \
        -o "$output_file" \
        --pdf-engine=xelatex \
        --highlight-style=tango \
        -V geometry:a4paper \
        -V geometry:"left=2cm,right=1.5cm,top=1.5cm,bottom=1.5cm" \
        -V mainfont="Liberation Sans" \
        -V monofont="Liberation Mono" \
        -V fontsize=10pt \
        -V monofontsize=9pt \
        -V colorlinks=true \
        -V linkcolor=blue \
        -V urlcolor=blue \
        --metadata title="Design Patterns trong ROS2" \
        --metadata author="ROS2 Design Pattern Guide" \
        --metadata date="$(date +%Y-%m-%d)" \
        -H header.tex
}

# Chuyển đổi các file markdown
convert_markdown_to_pdf "patterns/creational/abstract_factory.md" "output/1_abstract_factory.pdf"
convert_markdown_to_pdf "patterns/creational/factory.md" "output/2_factory.pdf"
convert_markdown_to_pdf "patterns/creational/singleton.md" "output/3_singleton.pdf"
convert_markdown_to_pdf "patterns/structural/adapter.md" "output/4_adapter.pdf"
convert_markdown_to_pdf "patterns/structural/bridge.md" "output/5_bridge.pdf"
convert_markdown_to_pdf "patterns/behavioral/observer.md" "output/6_observer.pdf"
convert_markdown_to_pdf "patterns/behavioral/strategy.md" "output/7_strategy.pdf"
convert_markdown_to_pdf "patterns/creational/builder.md" "output/8_builder.pdf"

# Ghép các file PDF lại
pdftk output/[1-8]_*.pdf cat output output/design_patterns_ros2.pdf

# Dọn dẹp
rm output/[1-8]_*.pdf
rm header.tex

echo "PDF đã được tạo tại output/design_patterns_ros2.pdf"