import { Container, AppBar, Toolbar, Typography, CssBaseline, ThemeProvider, createTheme } from '@mui/material';
import { ConnectionButton } from './components/ConnectionButton';
import { RobotDataDisplay } from './components/RobotDataDisplay';

const theme = createTheme({
  palette: {
    mode: 'dark',
    primary: {
      main: '#90caf9',
    },
    secondary: {
      main: '#f48fb1',
    },
  },
});

function App() {
  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <AppBar position="static">
        <Toolbar>
          <Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>
            Robot Monitor
          </Typography>
          <ConnectionButton />
        </Toolbar>
      </AppBar>
      <Container maxWidth="xl" sx={{ mt: 4 }}>
        <RobotDataDisplay />
      </Container>
    </ThemeProvider>
  );
}

export default App;
