import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import ComparisonSection from '@site/src/components/ComparisonSection';
import AdditionalContent from '@site/src/components/AdditionalContent';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className={clsx('container', styles.heroContainer)}>
        <div className={styles.heroText}>
          <Heading as="h1" className="hero__title">
            Physical AI & Humanoid Robotics
          </Heading>
          <p className="hero__subtitle">
            A Journey into Embodied Intelligence. Start building robots that don't just think, but *do*.
          </p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/intro">
              Start Reading Now 🚀
            </Link>
          </div>
        </div>
        <div className={styles.heroImage}>
          <img src="/img/home_page.jpg" alt="Book Cover" className={styles.bookCoverImage} />
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="An AI-Native Textbook for the next generation of AI and Robotics engineers.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <ComparisonSection />
        <AdditionalContent />
      </main>
    </Layout>
  );
}
