// @ts-check
const prismThemes = require('prism-react-renderer').themes;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Humanoid Robotics Book',
  tagline: 'Physical AI & Humanoid Robotics — From Simulation to Real Systems',
  favicon: 'img/favicon.ico',

  url: 'physical-ai-hackathon1-stq4.vercel.app',
  baseUrl: '/',

  organizationName: 'SIDRATALIB7878',
  projectName: 'humanoid-robotics-book',

  onBrokenLinks: 'ignore',
  onBrokenMarkdownLinks: 'ignore',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: 'docs',
          editUrl:
            'https://github.com/SIDRATALIB7878',
          
          showLastUpdateAuthor: true,
          showLastUpdateTime: true,
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/social-card.png',

    navbar: {
      title: 'Humanoid Robotics',
      logo: {
        alt: 'Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      hideOnScroll: true,
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'mainSidebar',
          position: 'left',
          label: 'Docs',
        },
        {
          to: '/chat',
          label: 'Chatbot',
          position: 'left',
        },

        // Right side (Author & Socials)
        {
          href: 'https://github.com/SIDRATALIB7878',
          label: 'GitHub',
          position: 'right',
        },
        {
          href: 'https://www.linkedin.com/in/sidra-talib/',
          label: 'LinkedIn',
          position: 'right',
        },
        {
          href: 'https://x.com/SIDRATALIB7878',
          label: 'X',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Content',
          items: [
            { label: 'All Chapters', to: '/docs' },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/SIDRATALIB7878/physical_Ai_Hackathon1',
            },
            {
              label: 'Author on LinkedIn',
              href: 'https://www.linkedin.com/in/sidra-talib/',
            },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Sidra Talib. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.vsLight,
      darkTheme: prismThemes.vsDark,
      additionalLanguages: ['python', 'cpp', 'java', 'javascript', 'bash'],
    },

    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    announcementBar: {
      id: 'github-star',
      content:
        '⭐ Support this project by starring it on <a target="_blank" rel="noopener noreferrer" href="https://github.com/SIDRATALIB7878/physical_Ai_Hackathon1">GitHub</a>',
      backgroundColor: '#467c85ff',
      textColor: '#ffffff',
      isCloseable: true,
    },
  },
};

module.exports = config;
