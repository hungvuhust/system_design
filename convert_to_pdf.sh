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

# Tạo file header.tex cho font configuration
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
\usepackage{fancyvrb}
\fvset{fontsize=\small}
\DefineVerbatimEnvironment{Highlighting}{Verbatim}{commandchars=\\\{\},fontsize=\small}
EOF

# Thêm trang bìa
cat << 'EOF' > $TMP_FILE
# Design Patterns trong ROS2 và Robotics

## Tài liệu tổng hợp về Design Patterns và ứng dụng trong ROS2

### Mục lục

EOF

# Thêm README.md vào đầu file (bỏ qua phần header và mục lục)
tail -n +12 README.md >> $TMP_FILE

# Thêm phân cách section
echo -e "\n\n---\n\n# Phần 1: Creational Patterns\n" >> $TMP_FILE

# Thêm các Creational Patterns
echo -e "\n## Abstract Factory Pattern\n" >> $TMP_FILE
cat patterns/creational/abstract_factory.md >> $TMP_FILE

echo -e "\n## Builder Pattern\n" >> $TMP_FILE
cat patterns/creational/builder.md >> $TMP_FILE

echo -e "\n## Factory Pattern\n" >> $TMP_FILE
cat patterns/creational/factory.md >> $TMP_FILE

echo -e "\n## Prototype Pattern\n" >> $TMP_FILE
cat patterns/creational/prototype.md >> $TMP_FILE

echo -e "\n## Singleton Pattern\n" >> $TMP_FILE
cat patterns/creational/singleton.md >> $TMP_FILE

# Thêm phân cách section
echo -e "\n\n---\n\n# Phần 2: Structural Patterns\n" >> $TMP_FILE

# Thêm các Structural Patterns
echo -e "\n## Adapter Pattern\n" >> $TMP_FILE
cat patterns/structural/adapter.md >> $TMP_FILE

echo -e "\n## Bridge Pattern\n" >> $TMP_FILE
cat patterns/structural/bridge.md >> $TMP_FILE

echo -e "\n## Composite Pattern\n" >> $TMP_FILE
cat patterns/structural/composite.md >> $TMP_FILE

echo -e "\n## Decorator Pattern\n" >> $TMP_FILE
cat patterns/structural/decorator.md >> $TMP_FILE

echo -e "\n## Facade Pattern\n" >> $TMP_FILE
cat patterns/structural/facade.md >> $TMP_FILE

echo -e "\n## Flyweight Pattern\n" >> $TMP_FILE
cat patterns/structural/flyweight.md >> $TMP_FILE

echo -e "\n## Proxy Pattern\n" >> $TMP_FILE
cat patterns/structural/proxy.md >> $TMP_FILE

# Thêm phân cách section
echo -e "\n\n---\n\n# Phần 3: Behavioral Patterns\n" >> $TMP_FILE

# Thêm các Behavioral Patterns
echo -e "\n## Chain of Responsibility Pattern\n" >> $TMP_FILE
cat patterns/behavioral/chain_of_responsibility.md >> $TMP_FILE

echo -e "\n## Command Pattern\n" >> $TMP_FILE
cat patterns/behavioral/command.md >> $TMP_FILE

echo -e "\n## Interpreter Pattern\n" >> $TMP_FILE
cat patterns/behavioral/interpreter.md >> $TMP_FILE

echo -e "\n## Iterator Pattern\n" >> $TMP_FILE
cat patterns/behavioral/iterator.md >> $TMP_FILE

echo -e "\n## Mediator Pattern\n" >> $TMP_FILE
cat patterns/behavioral/mediator.md >> $TMP_FILE

echo -e "\n## Memento Pattern\n" >> $TMP_FILE
cat patterns/behavioral/memento.md >> $TMP_FILE

echo -e "\n## Observer Pattern\n" >> $TMP_FILE
cat patterns/behavioral/observer.md >> $TMP_FILE

echo -e "\n## State Pattern\n" >> $TMP_FILE
cat patterns/behavioral/state.md >> $TMP_FILE

echo -e "\n## Strategy Pattern\n" >> $TMP_FILE
cat patterns/behavioral/strategy.md >> $TMP_FILE

echo -e "\n## Template Method Pattern\n" >> $TMP_FILE
cat patterns/behavioral/template_method.md >> $TMP_FILE

echo -e "\n## Visitor Pattern\n" >> $TMP_FILE
cat patterns/behavioral/visitor.md >> $TMP_FILE

# Tạo PDF với pandoc
echo "Đang tạo PDF..."
pandoc $TMP_FILE \
    -f markdown \
    -t pdf \
    --toc \
    --toc-depth=3 \
    --highlight-style=tango \
    --pdf-engine=xelatex \
    -V geometry:margin=1in \
    -V documentclass=report \
    -V fontsize=11pt \
    -V colorlinks=true \
    -V linkcolor=blue \
    -V urlcolor=blue \
    -H output/header.tex \
    -o $FINAL_PDF

# Xóa files tạm
rm $TMP_FILE
rm output/header.tex

echo "Đã tạo file PDF tại: $FINAL_PDF"