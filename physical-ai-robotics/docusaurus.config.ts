import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Robotics',
  tagline: 'A textbook for the next generation of AI and Robotics engineers',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // 🔥 Your GitHub Pages URL
  url: 'https://tehreem-asghar.github.io',

  // 🔥 Your repo name as base URL
  baseUrl: '/https://physical-ai-robotic-book-hackathon.vercel.app/',

  // 🔥 GitHub configuration
  organizationName: 'Tehreem-Asghar',
  projectName: 'physical_ai_robotic_book_hackathon',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/Tehreem-Asghar/physical_ai_robotic_book_hackathon/tree/main/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Robotics',
      logo: {
        alt: 'My Site Logo',
        src: 'img/logo.svg',
      },
      items: [],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Robotics, Inc. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;






















// import {themes as prismThemes} from 'prism-react-renderer';
// import type {Config} from '@docusaurus/types';
// import type * as Preset from '@docusaurus/preset-classic';

// // This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// const config: Config = {
//   title: 'Physical AI & Robotics',
//   tagline: 'A textbook for the next generation of AI and Robotics engineers',
//   favicon: 'img/favicon.ico',

//   // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
//   future: {
//     v4: true, // Improve compatibility with the upcoming Docusaurus v4
//   },

//   // Set the production url of your site here
//   url: 'https://tehreem-asghar.github.io',
//   // Set the /<baseUrl>/ pathname under which your site is served
//   // For GitHub pages deployment, it is often '/<projectName>/'
//   baseUrl: '/physical_ai_robotic_book_hackathon/',

//   // GitHub pages deployment config.
//   // If you aren't using GitHub pages, you don't need these.
//   organizationName: 'Tehreem-Asghar', // Usually your GitHub org/user name.
//   projectName: 'physical_ai_robotic_book_hackathon', // Usually your repo name.

//   onBrokenLinks: 'throw',

//   // Even if you don't use internationalization, you can use this field to set
//   // useful metadata like html lang. For example, if your site is Chinese, you
//   // may want to replace "en" with "zh-Hans".
//   i18n: {
//     defaultLocale: 'en',
//     locales: ['en'],
//   },

//   presets: [
//     [
//       'classic',
//       {
//         blog: {
//           showReadingTime: true,
//           feedOptions: {
//             type: ['rss', 'atom'],
//             xslt: true,
//           },
//           // Please change this to your repo.
//           // Remove this to remove the "edit this page" links.
//           editUrl:
//             'https://github.com/ai-robotics/physical-ai-robotics/tree/main/',
//           // Useful options to enforce blogging best practices
//           onInlineTags: 'warn',
//           onInlineAuthors: 'warn',
//           onUntruncatedBlogPosts: 'warn',
//         },
//         theme: {
//           customCss: './src/css/custom.css',
//         },
//       } satisfies Preset.Options,
//     ],
//   ],

//   themeConfig: {
//     // Replace with your project's social card
//     image: 'img/docusaurus-social-card.jpg',
//     colorMode: {
//       respectPrefersColorScheme: true,
//     },
//     navbar: {
//       title: 'Physical AI & Robotics',
//       logo: {
//         alt: 'My Site Logo',
//         src: 'img/logo.svg',
//       },
//       items: [
//         // {
//         //   type: 'docSidebar',
//         //   sidebarId: 'defaultSidebar',
//         //   position: 'left',
//         //   label: 'Tutorial',
//         // },
//         // {to: '/blog', label: 'Blog', position: 'left'},
//         // {
//         //   href: 'https://github.com/ai-robotics/physical-ai-robotics',
//         //   label: 'GitHub',
//         //   position: 'right',
//         // },
//       ],
//     },
//     footer: {
//       style: 'dark',
//       // links: [
//       //   {
//       //     title: 'Docs',
//       //     items: [
//       //       {
//       //         label: 'Tutorial',
//       //         to: '/docs/intro',
//       //       },
//       //     ],
//       //   },
//       //   // {
//       //   //   title: 'Community',
//       //   //   items: [
//       //   //     {
//       //   //       label: 'Stack Overflow',
//       //   //       href: 'https://stackoverflow.com/questions/tagged/docusaurus',
//       //   //     },
//       //   //     {
//       //   //       label: 'Discord',
//       //   //       href: 'https://discordapp.com/invite/docusaurus',
//       //   //     },
//       //   //     {
//       //   //       label: 'X',
//       //   //       href: 'https://x.com/docusaurus',
//       //   //     },
//       //   //   ],
//       //   // },
//       //   {
//       //     title: 'More',
//       //     items: [
//       //       {
//       //         label: 'Blog',
//       //         to: '/blog',
//       //       },
//       //       {
//       //         label: 'GitHub',
//       //         href: 'https://github.com/ai-robotics/physical-ai-robotics',
//       //       },
//       //     ],
//       //   },
//       // ],
//       copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Robotics, Inc. Built with Docusaurus.`,
//     },
//     prism: {
//       theme: prismThemes.github,
//       darkTheme: prismThemes.dracula,
//     },
//   } satisfies Preset.ThemeConfig,
// };

// export default config;
