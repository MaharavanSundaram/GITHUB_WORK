import os

def split_tex_sections(input_tex_file, output_folder):
    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    with open(input_tex_file, 'r',encoding='utf-8') as f:
        content = f.read()

        # Split the content based on '\section'
        sections = content.split('\\section')

        for i, section in enumerate(sections[1:]):  # Skipping the first element as it is empty
            # Create a new TeX file for each section
            section_file_path = os.path.join(output_folder, f'section_{i + 1}.tex')

            with open(section_file_path, 'w',encoding='utf-8') as section_file:
                # Write the section content to the new file
                section_file.write('\\documentclass{article}\n')
                section_file.write('\\begin{document}\n')
                section_file.write(f'\\section{section}\n')
                section_file.write('\\end{document}')

if __name__ == "__main__":
    input_tex_file = 'BST-DHW-AN013-00.tex'  # Replace with your input LaTeX file
    output_folder = 'output_sections'  # Replace with your desired output folder

    split_tex_sections(input_tex_file, output_folder)
