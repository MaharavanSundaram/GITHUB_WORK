import os

def sanitize_folder_name(name):
    # Remove unwanted characters from folder names
    return ''.join(c for c in name if c.isalnum() or c.isspace())

def split_tex_sections(input_tex_file, output_folder):
    # Create the output folder if it doesn't exist
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    with open(input_tex_file, 'r', encoding='utf-8') as f:
        content = f.read()

        # Split the content based on '\section'
        sections = content.split('\\section')

        for i, section in enumerate(sections[1:]):  # Skipping the first element as it is empty
            # Extract the section name
            section_name = section.split('\n', 1)[0].strip()

            # Create a sanitized folder name for the section
            section_folder_name = sanitize_folder_name(section_name)
            section_folder = os.path.join(output_folder, f'{i+1}_{section_folder_name}')
            os.makedirs(section_folder, exist_ok=True)

            # Create a file for the main section content (section.tex)
            section_file_path = os.path.join(section_folder, 'section.tex')

            with open(section_file_path, 'w', encoding='utf-8') as section_file:
                # Write the main section content to the new file
                section_file.write('\\documentclass{article}\n')
                section_file.write('\\begin{document}\n')
                section_file.write(f'\\section{section}\n')  # Main section content
                section_file.write('\\end{document}')

                # Split the section content based on '\subsection'
                subsections = section.split('\\subsection')

                for j, subsection in enumerate(subsections[1:]):  # Skipping the first element as it is main section content
                    # Extract the subsection name
                    subsection_name = subsection.split('\n', 1)[0].strip()

                    # Create a sanitized folder name for the subsection
                    subsection_folder_name = sanitize_folder_name(subsection_name)
                    subsection_folder = os.path.join(section_folder, f'subsection_{j + 1}_{subsection_folder_name}')
                    os.makedirs(subsection_folder, exist_ok=True)

                    # Create a new TeX file for each subsection
                    subsection_file_path = os.path.join(subsection_folder, 'subsection.tex')

                    with open(subsection_file_path, 'w', encoding='utf-8') as subsection_file:
                        # Write the subsection content to the new file
                        subsection_file.write('\\documentclass{article}\n')
                        subsection_file.write('\\begin{document}\n')
                        subsection_file.write(f'\\subsection{subsection}\n')
                        subsection_file.write('\\end{document}')

if __name__ == "__main__":
    input_tex_file = 'BST-DHW-AN013-00.tex'  # Replace with your input LaTeX file
    output_folder = 'output_sections'  # Replace with your desired output folder name

    split_tex_sections(input_tex_file, output_folder)
