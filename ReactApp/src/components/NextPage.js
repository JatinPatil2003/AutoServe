import React, { useState } from 'react';
import NavigationPage from './NavigationPage';
import MappingPage from './MappingPage';

function NextPage({ onBack }) {
  const [operation, setOperation] = useState(null);

  if (operation === 'navigation') {
    return <NavigationPage onBack={() => { setOperation(null); onBack(); }} />;
  }

  if (operation === 'mapping') {
    return <MappingPage onBack={() => { setOperation(null); onBack(); }} />;
  }

  return (
    <div>
      <h1>Robot Operations</h1>
      <button onClick={() => setOperation('mapping')}>Start Mapping</button>
      <button onClick={() => setOperation('navigation')}>Start Navigation</button>
      <button onClick={onBack}>Back</button>
    </div>
  );
}

export default NextPage;
