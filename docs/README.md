# Documentation Development

This directory contains the source files for the MkDocs-based documentation site.

## Local Development

### Prerequisites

Install the required Python packages:

```bash
pip install -r requirements.txt
```

### Preview the Documentation

To preview the documentation locally:

```bash
mkdocs serve
```

Then open your browser to `http://127.0.0.1:8000/`

The site will automatically reload when you make changes to the documentation files.

### Build the Site

To build the static site:

```bash
mkdocs build
```

The built site will be in the `site/` directory.

## Structure

```
docs/
├── index.md                    # Home page
├── getting-started/
│   ├── installation.md         # Installation guide
│   └── docker.md              # Docker setup
├── tutorials/
│   ├── r2c_overview.md        # ros2_control overview
│   ├── urdf_tutorial.md       # URDF description
│   ├── launch_tutorial.md     # Launch & interaction
│   ├── hardware_tutorial.md   # Hardware interface
│   ├── controller_tutorial.md # Controller development
│   ├── gazebo_tutorial.md     # Gazebo simulation
│   └── ethercat_tutorial.md   # EtherCAT integration
├── about/
│   └── contacts.md            # Contact information
└── images/                     # Image assets
```

## Adding Content

### Adding a New Page

1. Create a new Markdown file in the appropriate directory
2. Add the page to the navigation in `mkdocs.yml`
3. Preview your changes with `mkdocs serve`

### Adding Images

Place images in `docs/images/` and reference them using relative paths:

```markdown
![Alt text](../images/your-image.png)
```

### Using Admonitions

```markdown
!!! note "Optional Title"
    This is a note admonition.

!!! tip
    This is a tip.

!!! warning
    This is a warning.

!!! info
    This is an info box.
```

### Code Blocks

Use fenced code blocks with language specification:

````markdown
```python
def hello_world():
    print("Hello, World!")
```
````

## Deployment

The documentation is automatically deployed to GitHub Pages when changes are pushed to the `main` branch.

The deployment is handled by the GitHub Actions workflow in `.github/workflows/deploy-docs.yml`.

## Theme

The documentation uses the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme with:

- Dark/light mode toggle
- Blue primary and accent colors
- Navigation tabs and sections
- Search functionality
- Code syntax highlighting
- Responsive design

## Useful Links

- [MkDocs Documentation](https://www.mkdocs.org/)
- [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/)
- [Markdown Guide](https://www.markdownguide.org/)
