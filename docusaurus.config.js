// @ts-check
// Note: type annotations allow type checking and IDE autocompletion

const path = require('path');
const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulated Brains to Embodied Intelligence',
  favicon: 'img/logo.svg',

  // Set the production url of your site here
  url: 'https://physical-ai-and-humanoid-robotics-t-psi.vercel.app', // Vercel URL
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/', // Root path for Vercel deployment

  // GitHub pages deployment config (only used for GitHub Pages deployment)
  organizationName: 'Mirza-Usman-247', // GitHub org/user name
  projectName: 'physical-ai-and-humanoid-robotics-textbook', // Repo name

  onBrokenLinks: 'warn', // Changed to 'warn' for Phase 0 - will change to 'throw' after all modules created

  // Markdown configuration (migrated from deprecated onBrokenMarkdownLinks)
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  // Internationalization (i18n) - English only for now
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Edit URL for GitHub
          editUrl: 'https://github.com/Mirza-Usman-247/physical-ai-and-humanoid-robotics-textbook/tree/main/',
          // Show last update time and author
          showLastUpdateAuthor: true,
          showLastUpdateTime: true,
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
        // Sitemap generation
        sitemap: {
          changefreq: 'weekly',
          priority: 0.5,
          ignorePatterns: ['/tags/**'],
          filename: 'sitemap.xml',
        },
      }),
    ],
  ],

  stylesheets: [
    {
      href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
      type: 'text/css',
      integrity: 'sha384-0a4DDJ7ftLVUg7myaZ2vENw90bG7kZJlYU2F0JIzNvX+/f9RiCp4r6N3w2N7mZ',
      crossorigin: 'anonymous',
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // SEO metadata
      metadata: [
        {name: 'keywords', content: 'robotics, physical-ai, humanoid-robotics, ros2, isaac-sim, textbook'},
        {name: 'description', content: 'Comprehensive university-level textbook covering Physical AI and Humanoid Robotics'},
      ],

      // Social card for OpenGraph
      image: 'img/docusaurus-social-card.svg',

      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Textbook Logo',
          src: 'img/logo.svg',
          href: '/',
          target: '_self',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Textbook',
          },
          {
            href: 'https://github.com/Mirza-Usman-247/physical-ai-and-humanoid-robotics-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },

      footer: {
        style: 'dark',
        links: [
          {
            title: 'Core Modules',
            items: [
              {
                label: 'Module 0: Foundations',
                to: '/docs/module-0-foundations',
              },
              {
                label: 'Module 1: Kinematics & Control',
                to: '/docs/module-1-ros2',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/docs/module-2-digital-twin',
              },
            ],
          },
          {
            title: 'Advanced Modules',
            items: [
              {
                label: 'Module 3: NVIDIA Isaac',
                to: '/docs/module-3-isaac',
              },
              {
                label: 'Module 4: VLA & Humanoids',
                to: '/docs/module-4-vla-humanoids',
              },
              {
                label: 'Capstone Project',
                to: '/docs/capstone',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Glossary',
                to: '/docs/glossary',
              },
              {
                label: 'Hardware Lab Guide',
                to: '/docs/hardware-lab',
              },
              {
                label: 'Mathematical Notation',
                to: '/docs/notation',
              },
              {
                label: 'References',
                to: '/docs/references',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'About',
                to: '/docs/about',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/Mirza-Usman-247/physical-ai-and-humanoid-robotics-textbook',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook Team. Built with Docusaurus.`,
      },

      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'bash', 'json', 'yaml'],
      },

      // Algolia DocSearch configuration (to be configured later)
      // algolia: {
      //   appId: 'YOUR_APP_ID',
      //   apiKey: 'YOUR_SEARCH_API_KEY',
      //   indexName: 'physical-ai-textbook',
      //   contextualSearch: true,
      //   searchParameters: {
      //     facetFilters: ['module', 'week', 'difficulty_level'],
      //   },
      // },
    }),
};

module.exports = config;
