from latex2mathml.converter import convert

def latex_to_mathml(latex_code):
    mathml_code = convert(latex_code)
    return mathml_code

def generate_html_with_mathml(mathml_code):
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>LaTeX to HTML</title>
        <script type="text/javascript" async
            src="https://www.w3.org/Math/scripts/MathML3/mathml.js">
        </script>
    </head>
    <body>
        <h1>LaTeX to HTML</h1>
        <div id="mathml-content">
            {mathml_code}
        </div>
    </body>
    </html>
    """

    return html_template.format(mathml_code=mathml_code)

def main():
    # Read LaTeX content from file
    with open('BST-DHW-AN013-00.tex', 'r',encoding='utf-8') as file:
        latex_code = file.read()

    # Convert LaTeX to MathML
    mathml_code = latex_to_mathml(latex_code)

    # Generate HTML with embedded MathML
    html_code = generate_html_with_mathml(mathml_code)

    # Write HTML content to a file
    with open('output.html', 'w') as file:
        file.write(html_code)

if __name__ == "__main__":
    main()
